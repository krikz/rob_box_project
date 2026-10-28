"""Срез тулов хода: LLM видит ≤ 12 схем по снимку, словам человека и агенту (ADR-0150 §4.2, §2.1 п.5; #3696).

Функция, а не правило в промпте и не догадка LLM (подгрузка скиллов моделью удалена): всегда —
``knowledge.TOOLS_ALWAYS`` (речь, память, время, поиск, анимация); группы ``knowledge.TOOL_GROUPS`` — по словам
реплики («спой», «голос», «поезжай»…), затем сцены снимка (кадр ``DJ`` и играющая музыка — музыкальные тулы); ни одна
группа не совпала — группы прошлого хода (продолжение разговора), иначе ``TOOL_GROUPS_DEFAULT``. В SILENCED
ничего звучащего: срез пуст (LLM в тишине не зовётся). Провайдер без тулов — пусто. Слова с двумя владельцами («ускорь
темп», «жёстче») под играющей музыкой открывают music, а не voice; «я Саша» и «запомни мой голос» открывают people
(голос человека — не голос робота, ``knowledge.PERSON_VOICE_FORMS``, #4054); названия звуков («сирена») —
sound (#3976; таблицы — ``knowledge``). Открытый вопрос тула («Удалить «X»?», ``tool_ask``, #4062) держит группу
этого тула в срезе следующих ходов как названную. Голос (``set_voice`` /
``set_tts_provider``) виден только при явной голосовой просьбе в этой же реплике (#3744). Голый неслышимый ответ
(анимация или одна ``expression`` без слов), когда ни лицо словами не просили (:func:`face_asked`), ни ответа без слов
(:func:`silent_asked`, #4083), - :func:`needs_words` (#3995, #3997).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, List, Optional, Sequence, Tuple

from rob_box_core.tool_catalog import ToolCatalogEntry, get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog.agents import agent_spec, tools_for
from rob_box_dialog.expression import shows
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog import phrases
from rob_box_dialog.words import asks_robot_to_act, has_form, has_speech_manner, has_word, introduces_self, without_forms

if TYPE_CHECKING:  # только тип: срез тулов не тянет разбор команды
    from rob_box_dialog.command import Command


def groups_for(snapshot: SessionSnapshot, text: str, carry: Sequence[str] = (),
               asked: Sequence[str] = ()) -> Tuple[str, ...]:
    """Группы среза: прямо названные словами человека (порядок ``TOOL_GROUPS``) и группы тулов открытого вопроса
    (``asked``, :func:`asked_groups`, #4062), затем сцены снимка, затем названные только общим глаголом; ни одной —
    ``carry`` (группы прошлого хода: «я же просил про стрижей» — продолжение; кроме тех, что только по явной просьбе,
    ``TOOL_GROUPS_EXPLICIT``), иначе по умолчанию.

    Сцена идёт ПОСЛЕ названных групп: срез режется по ``TOOLS_VISIBLE_MAX``, и играющий диджей-сет (6 музыкальных
    схем + 6 всегда-видимых = 12) иначе вытеснял тулы, о которых человек спросил прямо — «сколько заряда» под музыку
    не видел ``get_battery_level``, и LLM отвечала «заряд мне не виден» (#3857, run 39849541397).
    """
    named = _under_music(snapshot, text, _named(text, "stems")) | set(asked_groups(asked))
    scene = ({"music"} if snapshot.music_playing else set()) - named
    weak = _named(text, "weak") - named - scene
    if not named | scene | weak:
        named = {g for g in carry if g in kn.TOOL_GROUPS and g not in kn.TOOL_GROUPS_EXPLICIT}
        named = named or set(kn.TOOL_GROUPS_DEFAULT)
    return _ordered(named) + _ordered(scene) + _ordered(weak)


def asked_groups(tools: Sequence[str]) -> Tuple[str, ...]:
    """Группы, где лежат тулы открытого вопроса (``tool_ask.asked_tools``, #4062): на «Робот, да, удаляй» слов
    группы нет, а тул спросил в прошлом ходе - его группа открыта, как названная (живой робот 27.10 14:47:
    ``music_library`` вне среза, модель «удалила» речью без вызова). Голос (``TOOL_GROUPS_EXPLICIT``) - только по
    словам человека."""
    return _ordered({g for g, group in kn.TOOL_GROUPS.items()
                     if g not in kn.TOOL_GROUPS_EXPLICIT and set(group.tools) & set(tools)})


def _under_music(snapshot: SessionSnapshot, text: str, named: set) -> set:
    """Неоднозначное слово («ускорь темп», «жёстче», «добавь бас») под играющей музыкой - про музыку, а не про голос.

    Сработать должны все три условия: музыка звучит (снимок, а не слова), в реплике слово из ``MUSIC_AMBIGUOUS_*``, и
    речь робота человек не назвал («говори быстрее», «читай помедленнее» остаются за голосом - ``SPEECH_OWNER_STEMS``).
    Тогда music попадает в названные группы, а voice уходит, если его открыла только скорость (``VOICE_PACE_STEMS``):
    «ускорь темп и смени голос» оставляет обе. В тишине таблица не действует - как раньше (n506, 23.10)."""
    if not snapshot.music_playing or has_word(text, kn.SPEECH_OWNER_STEMS):
        return named
    if not (has_word(text, kn.MUSIC_AMBIGUOUS_STEMS) or has_form(text, kn.MUSIC_AMBIGUOUS_WORDS)):
        return named
    if "voice" in named and not has_word(text, _voice_only_stems()):
        named = named - {"voice"}
    return named | {"music"}


def _voice_only_stems() -> Tuple[str, ...]:
    """Основы voice, которые называют голос однозначно (всё, кроме слов про скорость)."""
    return tuple(s for s in kn.TOOL_GROUPS["voice"].stems if s not in kn.VOICE_PACE_STEMS)


def requested_groups(text: str) -> Tuple[str, ...]:
    """Группы, названные словами человека в ЭТОЙ реплике: без сцен снимка, переноса и умолчания."""
    return _ordered(_named(text, "stems") | _named(text, "weak"))


def command_groups(text: str, called: Sequence[str]) -> Tuple[str, ...]:
    """Исполненная просьба человека: группы, названные его словами, в которых LLM вызвала тул-действие (класс
    ``TOOL_CLASSES`` не ``query``). Пусто — ход не команда.

    Решает запрос человека, тулы LLM — только подтверждение исполнения: чтение (статус, время, память, поиск,
    восприятие) и действие, о котором человек не просил (анимация к болтовне), командой ход не делают; слово группы
    в болтовне («узнал по голосу») без исполнения — тоже (В17, ADR-0148; живой робот 21.10).
    """
    actions = {t for t in called if kn.TOOL_CLASSES.get(t, "query") != "query"}
    return tuple(g for g in requested_groups(text) if actions & set(kn.TOOL_GROUPS[g].tools))


def needs_words(text: str, cmd: "Command") -> bool:
    """Ответ модели не слышен, а человек ждёт слов (#3995, #3997): нет ни речи, ни вопроса, ни выступления, ни
    ``passed``, все действия команды - неслышимые (``knowledge.INAUDIBLE_TOOLS``), а ответ есть - действия или одна
    ``expression`` (``expression.shows``: «покажи, как тебе грустно» → ``{emotion: sad}``; или один earcon - короткий
    сигнал, не слова: ревью #4041), и ни лицо человек словами не просил (группы ``FACE_ASK_GROUP`` нет в
    :func:`requested_groups`: «покажи анимацию радости», «улыбнись», «посмотри налево»), ни ответа без слов
    (:func:`silent_asked`, #4083: «покажи молча, что ты задумался» - лицом или анимацией одинаково, без речи).

    Чистая функция: решает по реплике и команде, а не по тексту ответа модели. Кто ждёт слов (агент с речью в
    динамики, адресованная реплика, не тишина), решает ход (``turn.words_owed``)."""
    if cmd.speech or cmd.ask is not None or cmd.perform is not None or cmd.passed:
        return False
    if any(a.tool not in kn.INAUDIBLE_TOOLS for a in cmd.actions):
        return False
    face = cmd.expression
    if not cmd.actions and not shows(face.emotion, face.gesture, face.face) and face.earcon is None:
        return False
    return not show_asked(text)


def silent_asked(text: str) -> bool:
    """Просил ли человек ответить без слов (#4083): форма из ``knowledge.SILENT_ASK_FORMS`` («молча», «без слов», «не
    говори») или «тихо» рядом с «покажи» (``SILENT_SHOW_FORMS`` + ``FACE_SHOW_STEMS``: «тихо покажи радость»). Целыми
    словами: «не молчи» и «помолчи» без слов не просят (тишину решает Tier-1). Таблица одна - ``knowledge``."""
    if has_form(text or "", kn.SILENT_ASK_FORMS):
        return True
    return has_form(text or "", kn.SILENT_SHOW_FORMS) and has_word(text or "", kn.FACE_SHOW_STEMS)


def robot_asked(text: str) -> bool:
    """Просил ли человек в чате сделать это РОБОТОМ (#4142 п.7, классы 1 и 3 #4040): форма из
    ``knowledge.CHAT_ROBOT_ASK_FORMS`` («покекай роботом», «через робота», «вслух», «развлекай народ»), не названная темой,
    и реплика не справка (ревью #4153, #4159 п.3: «Как управлять роботом?», «Почему детям полезно читать вслух?»,
    «найди, что пишет народ» - ответ в чат, робот ничего не делает). Целыми словами: «робота-курьера» - не «через
    робота». Решение - ``words.asks_robot_to_act``, таблицы - ``knowledge``."""
    return asks_robot_to_act(text or "")


def show_asked(text: str) -> bool:
    """Просил ли человек показать лицо: словами о лице (группа ``FACE_ASK_GROUP``, :func:`face_asked`) или ответа без
    слов (:func:`silent_asked`). Тогда ответ лицом без речи - норма (:func:`needs_words`), а лицо команды держится как
    показ по просьбе (``turn.command_face``, ``expression.show_seconds``: не меньше ``FACE_SHOW_MIN_S``)."""
    return kn.FACE_ASK_GROUP in requested_groups(text) or silent_asked(text)


def voice_asked(snapshot: SessionSnapshot, text: str, voices: Sequence[str] = ()) -> bool:
    """Просил ли человек в ЭТОЙ реплике о голосе (#4021, n109): его словами открыта группа «только по явной просьбе»
    (``knowledge.TOOL_GROUPS_EXPLICIT``: «голос», «яндекс», «басом»; под играющей музыкой «ускорь» - музыка, не голос,
    :func:`groups_for`) или названо имя голоса реестра (``voices``). Нет - голос реплики модели (``Command.voice``)
    закрыт формой хода: голос сам собой не меняется."""
    if set(groups_for(snapshot, text)) & set(kn.TOOL_GROUPS_EXPLICIT):
        return True
    return voice_named(text, voices)


def voice_named(text: str, voices: Sequence[str] = ()) -> bool:
    """Назван ли в ЭТОЙ реплике голос реестра по имени (``voices`` — id, название, алиасы, каждое целым словом): «спой
    голосом Алисы». Такой голос звучит только речью робота (TTS): путь пения с ним — живое выступление, а не генерация
    Suno, где голос молча потерялся бы (ревью #4153, ``turn.sing_route``). «Своим голосом» — не имя реестра."""
    return has_form(text or "", tuple(v for v in voices if v))


def face_asked(text: str) -> bool:
    """Просил ли человек словами показать лицо: глагол из ``FACE_ASK_STEMS`` («улыбнись», «изобрази») или «покажи»
    рядом с тем, что показать (``FACE_SHOW_WHAT_STEMS``: «покажи победу», «покажи, как тебе грустно»). Голое «покажи»
    («покажи кухню») лицо не просит. Таблица одна - ``knowledge``; группа ``FACE_ASK_GROUP`` открывается по ней же."""
    if has_word(text or "", kn.FACE_ASK_STEMS):
        return True
    return has_word(text or "", kn.FACE_SHOW_STEMS) and has_word(text or "", kn.FACE_SHOW_WHAT_STEMS)


def motion_named(text: str) -> bool:
    """Назвал ли человек движение базы глаголом («поезжай», «двигайся», «повернись»). Восприятие («посмотри налево»)
    и голые направления движение не открывают; «повернись глазами» — взгляд, а не поворот базы (#3779)."""
    if has_word(text or "", kn.MOTION_VERB_STEMS):
        return True
    return has_word(text or "", kn.MOTION_TURN_STEMS) and not has_word(text or "", kn.MOTION_EYES_STEMS)


def motion_refusal(tool: str, text: str) -> Optional[str]:
    """Честная фраза отказа, если тул движения вызван в ходе, где человек движение не называл; иначе ``None``.

    Страховка исполнителя поверх среза: что бы ни написала LLM, база без глагола движения не поедет. Пустая реплика
    (вызов не по голосу человека) отказа не даёт — судить не по чему."""
    if tool not in kn.MOTION_TOOLS or not (text or "").strip() or motion_named(text):
        return None
    return phrases.text("motion.rejected.not_asked")


def widen_for(missing: Sequence[str], agent: str, have: Sequence[str]) -> Tuple[str, ...]:
    """Тулы, которые код добавляет в запрос, когда модель позвала тул вне среза (``tool_not_available``, #3960): тул
    есть в реестре агента, а слова человека его группу не открыли («поставь ровный ритм» - живой прогон 23.10, акт 8).
    Открывается вся группа названного тула - по членству или по имени группы в имени тула (опечатка ``music_stop`` →
    группа ``music``); тула нет ни в группе, ни в реестре - ничего (выбор сужает код, угадывания по тексту нет).
    Не открываются: движение (только по глаголу человека, ``motion_named``) и голос (только по явной просьбе, #3744).
    Уже видимые (``have``) не повторяются; порядок - как в группе, названный тул первым."""
    allowed = set(tools_for(agent_spec(agent)))
    opened: List[str] = []
    for tool in missing:
        opened.extend(n for n in _group_of(tool, allowed) if n not in have and n not in opened)
    return tuple(opened)


def _group_of(tool: str, allowed: set) -> List[str]:
    """Тул и его группа (по членству или по имени группы в имени тула), без того, что модель открыть не может."""
    if tool in kn.MOTION_TOOLS:  # база едет по глаголу человека (``motion_named``), а не по просьбе модели
        return []
    groups = [g for g, group in kn.TOOL_GROUPS.items()
              if g not in kn.TOOL_GROUPS_EXPLICIT and (tool in group.tools or g in tool.lower())]
    names = [tool] + [t for g in groups for t in kn.TOOL_GROUPS[g].tools]
    closed = set(kn.MOTION_TOOLS) | set(kn.TOOLS_ALWAYS) | set(kn.TOOL_GROUPS["voice"].tools)
    return [n for n in names if n in allowed and n not in closed]


def _named(text: str, field: str) -> set:
    said, own_voice = without_forms(text, kn.PERSON_VOICE_FORMS)  # «запомни мой голос» - не голос робота (#4054)
    names = {name for name, group in kn.TOOL_GROUPS.items() if has_word(said, getattr(group, field))}
    if field == "stems" and (own_voice or introduces_self(text)):  # «здравствуй, я Саша»: основой это не поймать
        names.add(kn.SELF_INTRO_GROUP)
    if field == "stems" and face_asked(text):  # «покажи победу»: «покажи» без «победы» лицо не просит
        names.add(kn.FACE_ASK_GROUP)
    if field == "stems" and has_speech_manner(text):  # «хватит тянуть резину» - про темп речи (#4074)
        names.add(kn.VOICE_STEP_GROUP)
    return names


def _ordered(names: set) -> Tuple[str, ...]:
    return tuple(name for name in kn.TOOL_GROUPS if name in names)


def available_tools(snapshot: SessionSnapshot, caps: Optional[Any] = None, agent: str = "personality",
                    text: str = "", carry: Sequence[str] = (),
                    asked: Sequence[str] = ()) -> Tuple[ToolCatalogEntry, ...]:
    """Тулы хода (≤ ``TOOLS_VISIBLE_MAX``) из среза агента: всегда-видимые, затем группы по приоритету.

    ``caps`` — возможности провайдера (``tools=False`` — тулов нет вовсе); ``None`` — не известны, тулы есть.
    ``carry`` — группы прошлого хода, ``asked`` — тулы открытого вопроса (см. :func:`groups_for`).
    """
    if snapshot.top("Silence") is not None or (caps is not None and not getattr(caps, "tools", True)):
        return ()
    allowed = set(tools_for(agent_spec(agent))) | set(kn.SPEECH_CHANNEL_TOOLS)
    names: List[str] = []
    drive = motion_named(text)
    grouped = tuple(t for g in groups_for(snapshot, text, carry, asked) for t in kn.TOOL_GROUPS[g].tools)
    for name in kn.TOOLS_ALWAYS + kn.ALOUD_TOOLS + grouped:
        if name in kn.MOTION_TOOLS and not drive:
            continue
        if name in allowed and name not in names and len(names) < kn.TOOLS_VISIBLE_MAX:
            names.append(name)
    return tuple(get_tool(name) for name in names)


__all__ = ["asked_groups", "available_tools", "command_groups", "face_asked", "groups_for", "motion_named",
           "motion_refusal", "needs_words", "requested_groups", "robot_asked", "show_asked", "silent_asked",
           "voice_asked", "voice_named", "widen_for"]
