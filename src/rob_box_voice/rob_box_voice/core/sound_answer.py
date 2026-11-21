"""Режим «отвечай звуками» (ADR-0175 §1.2): решение хода, не второй путь звука.

Хост публикует набор клипов манифестом (идентификатор → файл, версия, sha256).
Код сопоставляет короткую реакцию с идентификатором и просит штатный
``play_sound`` (``/voice/sound/trigger`` → ``SoundEvent``). Файлов здесь нет:
пока каталог ``sound_answer_clip_dir`` пуст, владелец честно отвечает
``skipped{not_found}``, и фраза успеха из этого не строится.

Режим живёт в сессии разговора. «Стоп» рвёт текущий клип (штатный
``/voice/sound/stop``). «Обычным голосом» режим снимает; фразу об этом уже
строит ``set_voice``, вторая реплика не добавляется. Ответ с числом, именем
или причиной ошибки звуком не подменяется. Звуковая реакция не подтверждает
другое действие: «спел» / «сказал» — только из фактического ``finished``.
"""

from __future__ import annotations

import hashlib
import json
import os
import unicodedata
import uuid
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence, Tuple

from rob_box_dialog_msgs.msg import SoundEvent

#: Кадр сессии. В запиненном ядре v0.7.14 его нет в ``SCENE_KINDS`` — хост держит режим сам
#: и снимает его на ``reset`` (новая сессия: ``session_generation`` +1) и на «обычным голосом».
SCENE_KIND = "SoundAnswer"
#: Имя параметра ноды: каталог файлов манифеста. Пустой — пакета §0.1 ещё нет.
CLIP_DIR_PARAM = "sound_answer_clip_dir"
#: Имя параметра: путь манифеста. Путь по умолчанию — файл пакета, не чужой каталог.
MANIFEST_PARAM = "sound_answer_manifest"
#: Пять идентификаторов заглушки, пока пакета §0.1 нет. Других здесь не появляется.
STUB_IDS: Tuple[str, ...] = ("bird", "seal", "elephant", "alligator", "growl")
#: Роль реакции → идентификатор заглушки. Роль выбирает код, не модель.
ROLE_CLIPS: Mapping[str, str] = {
    "greeting": "bird",
    "surprise": "seal",
    "confirm": "elephant",
    "question": "alligator",
    "refuse": "growl",
}
#: Закрытые формы включения. Не регекс: сверка по словам после вейка.
ENABLE_FORMS: Tuple[Tuple[str, ...], ...] = (
    ("отвечай", "звуками"),
    ("отвечай", "мне", "звуками"),
    ("говори", "звуками"),
    ("говори", "мне", "звуками"),
)
#: Закрытые формы снятия. «Обычным голосом» уже команда ``set_voice`` — здесь только снятие режима.
DISABLE_FORMS: Tuple[Tuple[str, ...], ...] = (
    ("обычным", "голосом"),
    ("верни", "обычный", "голос"),
    ("верни", "свой", "обычный", "голос"),
    ("говори", "обычным", "голосом"),
)
#: «Стоп» текущего клипа — те же ясные формы, что гасят звук, и ничего больше.
STOP_FORMS: Tuple[str, ...] = ("стоп", "стой", "хватит", "остановись", "отставить")
#: Число словами — тоже точный текст. Закрытый список, не регекс: цифра ловится отдельно, по знаку.
NUMBER_WORDS: Tuple[str, ...] = (
    "ноль",
    "один",
    "одна",
    "одно",
    "два",
    "две",
    "три",
    "четыре",
    "пять",
    "шесть",
    "семь",
    "восемь",
    "девять",
    "десять",
    "одиннадцать",
    "двенадцать",
    "тринадцать",
    "четырнадцать",
    "пятнадцать",
    "шестнадцать",
    "семнадцать",
    "восемнадцать",
    "девятнадцать",
    "двадцать",
    "тридцать",
    "сорок",
    "пятьдесят",
    "шестьдесят",
    "семьдесят",
    "восемьдесят",
    "девяносто",
    "сто",
    "двести",
    "триста",
    "четыреста",
    "пятьсот",
    "шестьсот",
    "семьсот",
    "восемьсот",
    "девятьсот",
    "тысяча",
    "тысячи",
    "тысяч",
    "миллион",
    "миллиона",
    "миллионов",
)
#: Служебные слова вокруг команды: «ну отвечай звуками пожалуйста».
_FILLER = frozenset(
    {
        "ну",
        "а",
        "и",
        "же",
        "пожалуйста",
        "давай",
        "мне",
        "ты",
        "вы",
        "робот",
        "сейчас",
    }
)
#: Слова причины ошибки: точный текст, звук его не подменяет.
_ERROR_FORMS = frozenset(
    {
        "ошибка",
        "ошибки",
        "ошибку",
        "ошибке",
        "ошибкой",
        "не",
        "вышло",
        "не удалось",
        "недоступен",
        "недоступна",
        "недоступно",
        "отказ",
        "отказано",
        "отказался",
        "сбой",
        "сбоя",
    }
)
#: Подпись клипа в данных действия. Фразу успеха из неё строить нельзя — только из ``finished``.
CLIP_FIELD = "clip_id"
FINISHED_FIELD = "playback"
FINISHED_VALUE = "finished"

_KIND_FINISHED = int(getattr(SoundEvent, "KIND_FINISHED", 1))
_KIND_SKIPPED = int(getattr(SoundEvent, "KIND_SKIPPED", 2))
_NUMBER_WORDS = frozenset(NUMBER_WORDS)


@dataclass(frozen=True)
class Clip:
    """Клип набора: идентификатор, имя файла, роль реакции."""

    clip_id: str
    file: str
    role: str


@dataclass(frozen=True)
class Manifest:
    """Опубликованный набор: версия, sha256 и клипы. Пустой набор — честный «пакета нет»."""

    version: str
    sha256: str
    clips: Tuple[Clip, ...]

    def by_id(self, clip_id: str) -> Optional[Clip]:
        return next((c for c in self.clips if c.clip_id == clip_id), None)

    def by_role(self, role: str) -> Optional[Clip]:
        return next((c for c in self.clips if c.role == role), None)


@dataclass(frozen=True)
class AnswerChoice:
    """Решение хода: клип вместо речи, либо точный текст, который звуком не подменяется."""

    clip_id: Optional[str]
    precise: bool
    reason: str


def default_manifest() -> str:
    """Манифест-заглушка из пакета. На роботе его подменяет конфиг, смонтированный томом.

    Вне ROS-рантайма (тесты) — файл пакета рядом с модулем. Ноду звука за этим путём не
    импортируем: она на импорте требует ``audio_common_msgs``.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("rob_box_voice"), "config", "sound_answer_manifest.json")
    except Exception:  # noqa: BLE001 — ament нет (тесты, хост без рантайма)
        return os.path.join(os.path.dirname(__file__), "..", "..", "config", "sound_answer_manifest.json")


def load_manifest(path: str) -> Manifest:
    """Прочитать манифест. Битый файл и чужая сумма — пустой набор, не выдуманные клипы."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = handle.read()
        body = json.loads(raw)
    except (OSError, ValueError):
        return Manifest("", "", ())
    if not isinstance(body, dict):
        return Manifest("", "", ())
    clips = _clips(body.get("clips"))
    digest = str(body.get("sha256") or "")
    if digest and digest != hashlib.sha256(_canonical(clips).encode("utf-8")).hexdigest():
        return Manifest(str(body.get("version") or ""), digest, ())
    return Manifest(str(body.get("version") or ""), digest, clips)


def clip_path(clip_dir: str, clip: Clip) -> str:
    """Путь файла клипа в каталоге конфига. Каталог пуст — файла нет, и это не ошибка модуля."""
    return os.path.join(clip_dir, clip.file) if clip_dir else ""


def clip_file_ready(manifest_path: str, clip_dir: str, clip_id: str) -> bool:
    """Файл клипа лежит в каталоге конфига. Нет манифеста, файла или id — не готов, не выдуман."""
    clip = load_manifest(manifest_path).by_id(clip_id)
    return clip is not None and file_ready(clip_dir, clip)


def file_ready(clip_dir: str, clip: Clip) -> bool:
    """Файл клипа лежит там, куда указывает конфиг. Нет файла — играть нечего."""
    path = clip_path(clip_dir, clip)
    return bool(path) and os.path.isfile(path)


def classify(text: str) -> str:
    """Команда режима по словам реплики: ``enable`` / ``disable`` / ``stop`` / ``""``.

    Сверка по словам, не по регексу. Хвост из служебных слов команду не отменяет;
    чужое слово («отвечай звуками и спой») — уже не команда режима.
    """
    words = _words(text)
    if not words:
        return ""
    if _covered(words, ENABLE_FORMS):
        return "enable"
    if _covered(words, DISABLE_FORMS):
        return "disable"
    if len(words) == 1 and words[0] in STOP_FORMS:
        return "stop"
    return ""


def keeps_precise_text(text: str, known_names: Sequence[str] = ()) -> bool:
    """Ответ с числом, именем или причиной ошибки звуком не подменяется.

    Число — и цифра, и слово из закрытого списка (``NUMBER_WORDS``): «три метра» так же
    точны, как «3 метра». Имя сверяется по словам, не по подстроке.
    """
    words = _words(text)
    if any(any(ch.isdigit() for ch in word) for word in words):
        return True
    if any(word in _NUMBER_WORDS for word in words):
        return True
    if _error_reason(words):
        return True
    known = {_fold(name) for name in known_names if name}
    return any(word in known for word in words)


def choose(text: str, manifest: Manifest, known_names: Sequence[str] = ()) -> AnswerChoice:
    """Короткая реакция из набора либо точный текст. Пустой набор — клипа нет, текст остаётся."""
    if keeps_precise_text(text, known_names):
        return AnswerChoice(None, True, "precise")
    if not manifest.clips:
        return AnswerChoice(None, False, "no_catalog")
    clip = manifest.by_role(_role(text))
    if clip is None:
        return AnswerChoice(None, False, "no_clip")
    return AnswerChoice(clip.clip_id, False, clip.role)


def finished_phrase(kind: int, reason: str = "") -> Optional[str]:
    """«Сказал» — только из фактического ``finished``. ``started`` и ``skipped`` молчат.

    Пропуск не подтверждает действие: причины разные, фразы успеха нет ни у одной.
    """
    if int(kind) != _KIND_FINISHED or reason:
        return None
    return "Сказал."


def skipped(kind: int) -> bool:
    """Исход — пропуск, а не конец проигрывания."""
    return int(kind) == _KIND_SKIPPED


def _clips(raw: object) -> Tuple[Clip, ...]:
    if not isinstance(raw, list):
        return ()
    clips = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        clip_id, file, role = (str(item.get(key) or "").strip() for key in ("id", "file", "role"))
        if clip_id and file and role and _safe_file(file):
            clips.append(Clip(clip_id, file, role))
    return tuple(clips)


def _canonical(clips: Sequence[Clip]) -> str:
    rows = [{"id": c.clip_id, "file": c.file, "role": c.role} for c in clips]
    return json.dumps(rows, ensure_ascii=False, separators=(",", ":"), sort_keys=True)


def _safe_file(name: str) -> bool:
    """Имя файла клипа — один сегмент. Чужой путь из манифеста не принимается."""
    return name == os.path.basename(name) and ".." not in name


def _words(text: str) -> Tuple[str, ...]:
    folded = _fold(text)
    out = []
    token = []
    for char in folded:
        if char.isalnum() or char == "_":
            token.append(char)
        elif token:
            out.append("".join(token))
            token = []
    if token:
        out.append("".join(token))
    return tuple(out)


def _fold(text: str) -> str:
    return unicodedata.normalize("NFKC", text or "").lower().replace("ё", "е")


def _covered(words: Sequence[str], forms: Sequence[Sequence[str]]) -> bool:
    body = tuple(word for word in words if word not in _FILLER)
    return any(body == tuple(form) for form in forms)


def _error_reason(words: Sequence[str]) -> bool:
    if any(word in _ERROR_FORMS for word in words):
        return True
    pairs = zip(words, words[1:])
    return any(f"{left} {right}" in _ERROR_FORMS for left, right in pairs)


def _role(text: str) -> str:
    words = set(_words(text))
    if words & {"привет", "здравствуй", "здравствуйте"}:
        return "greeting"
    if words & {"ого", "ух"} or "!" in (text or ""):
        return "surprise"
    if words & {"нет", "нельзя", "откажу"}:
        return "refuse"
    if (text or "").rstrip().endswith("?"):
        return "question"
    return "confirm"


@dataclass
class Mode:
    """Режим одной сессии разговора. Новая сессия (``DialogControl.reset``) режима не наследует.

    ``reset`` эпохи поднимает, ``session_id`` оставляет, поэтому поколение сессии — отдельный
    счётчик: ``session_generation`` +1 на reset. ``replace_epoch`` (стоп) поколение не трогает —
    режим после стопа жив.
    """

    on: bool = False
    session_id: str = ""
    generation: int = 0

    def live(self, session_id: str, generation: int) -> bool:
        return self.on and self.session_id == session_id and self.generation == int(generation)


class SoundAnswerGate:
    """Решение хоста: клип вместо речи, точный текст остаётся речью, фраза успеха — только из ``finished``.

    Каталог читается с диска на каждое решение: конфиг смонтирован томом, пакет §0.1 подъедет без пересборки.
    """

    def __init__(
        self, manifest_path: str, clip_dir: str, known_names: Sequence[str] = (), name_source: Optional[Any] = None
    ) -> None:
        self.manifest_path = manifest_path
        self.clip_dir = clip_dir
        self.known_names = tuple(known_names)
        # Вызов без аргументов → имена, известные диалогу сейчас. Статический кортеж его не заменяет.
        self.name_source = name_source
        self.mode = Mode()
        self._pending: dict = {}

    def names(self) -> Tuple[str, ...]:
        """Имена, которые клипом не подменяются: статический список плюс то, что источник отдаёт сейчас."""
        fresh = self.name_source() if self.name_source is not None else ()
        return tuple(dict.fromkeys((*self.known_names, *(str(name) for name in fresh or () if name))))

    def observe(self, text: str, session_id: str, session_generation: int) -> str:
        """Команда человека до хода. ``stop`` режим не снимает: рвётся только текущий клип.

        Ожидание клипа ``stop`` тоже не чистит — это делает :meth:`stop_pending`, когда обрыв
        реально дошёл до владельца клипа. Команда без обрыва «Сказал.» не отменяет.
        """
        self._drop_if_new_session(session_id, session_generation)
        kind = classify(text)
        if kind == "enable":
            self.mode = Mode(True, session_id, int(session_generation))
        elif kind == "disable":
            self.mode = Mode(False, session_id, int(session_generation))
        return kind

    def active(self, session_id: str, session_generation: int) -> bool:
        return self.mode.live(session_id, session_generation)

    def instead_of_speech(self, text: str, session_id: str, session_generation: int, request_id: str) -> Optional[str]:
        """Идентификатор клипа вместо этой реплики, либо ``None`` — реплика звучит речью.

        Точный текст (число, имя, причина ошибки) клипом не подменяется и в пару к клипу не идёт.
        Клипа нет в каталоге или файла нет — тоже речь: врать «сказал» нечем.
        """
        if not self.mode.live(session_id, session_generation) or not (text or "").strip():
            return None
        if classify(text):
            return None
        choice = choose(text, load_manifest(self.manifest_path), self.names())
        if choice.clip_id is None or choice.precise:
            return None
        clip = load_manifest(self.manifest_path).by_id(choice.clip_id)
        if clip is None or not file_ready(self.clip_dir, clip):
            return None
        self._pending[request_id] = choice.clip_id
        return choice.clip_id

    def stop_pending(self) -> Tuple[str, ...]:
        """«Стоп» дошёл до владельца клипа: ждущие клипы больше не подтверждаются «Сказал.»."""
        pending = tuple(self._pending)
        self._pending.clear()
        return pending

    def trigger_payload(self, clip_id: str, request_id: str) -> str:
        """Тело ``/voice/sound/trigger``: тот же JSON, что у ``play_sound``."""
        return json.dumps({"sound": clip_id, "request_id": request_id}, ensure_ascii=False)

    def new_request_id(self) -> str:
        return uuid.uuid4().hex[:8]

    def on_finished(self, request_id: str, kind: int, reason: str = "") -> Optional[str]:
        """Фраза кода, когда клип этого режима доиграл. ``started`` ещё не конец — ожидание живёт.

        Чужой звук и любой пропуск молчат. Пропуск ожидание снимает: второй фразы из него не будет.
        """
        if request_id not in self._pending:
            return None
        if int(kind) not in (_KIND_FINISHED, _KIND_SKIPPED):
            return None
        self._pending.pop(request_id, None)
        return finished_phrase(kind, reason)

    def _drop_if_new_session(self, session_id: str, session_generation: int) -> None:
        """``reset`` поднимает поколение сессии при том же ``session_id`` — режим и ожидание снимаются."""
        if self.mode.session_id and (
            self.mode.session_id != session_id or int(session_generation) != self.mode.generation
        ):
            self.mode = Mode(False, session_id, int(session_generation))
            self._pending.clear()


class StopFlag:
    """Поколение «стоп». ``sd.stop()`` гасит и естественный конец, поэтому конец клипа им не доказывается.

    ``set`` до старта потока не теряется: ``arm`` флаг не снимает, ``aborted`` видит поколение,
    которое стоп уже поднял. ``clear`` остаётся для теста и для конца проигрывания, не для старта.
    """

    def __init__(self) -> None:
        self.generation = 0
        self._set = False

    def set(self) -> None:
        self._set = True
        self.generation += 1

    def clear(self) -> None:
        self._set = False

    def is_set(self) -> bool:
        return self._set


class SpeechAsClip:
    """``ports.say``: в режиме неточный ответ уходит клипом, точный — прежней речью, второй реплики нет.

    Хост ставит это вместо метода порта. ``stopped`` — поколение обрыва: «стоп» поднимает его, поток
    проигрывания сверяет и не публикует ``finished`` на оборванный клип.
    """

    def __init__(
        self, gate: SoundAnswerGate, say: Any, publish: Any, session: Any, stopped: Optional[Any] = None
    ) -> None:
        self.gate = gate
        self._say = say
        self._publish = publish
        self._session = session
        self.stopped = stopped if stopped is not None else StopFlag()
        self.clips: list = []
        self.spoken: list = []

    def __call__(
        self,
        turn_id: str,
        epoch: int,
        text: str,
        voice: Optional[str] = None,
        emotion: Optional[str] = None,
        boundary: str = "none",
    ) -> str:
        snap = self._session.snapshot
        request_id = self.gate.new_request_id()
        # Поколение сессии, не эпоха: стоп эпохи поднимает, режим после него жив (ADR-0175 §1.2).
        clip_id = self.gate.instead_of_speech(text, snap.session_id, _session_generation(snap), request_id)
        if clip_id is None:
            self.spoken.append(text)
            if boundary != "none":
                return self._say(turn_id, epoch, text, voice, emotion, boundary=boundary)
            return self._say(turn_id, epoch, text, voice, emotion)
        self.clips.append(clip_id)
        self._publish(self.gate.trigger_payload(clip_id, request_id))
        return ""

    def stop(self) -> None:
        """«Стоп» текущего клипа: поколение обрыва, по которому поток не скажет ``finished``."""
        self.stopped.set()

    def say_finished(self, turn_id: str, epoch: int, phrase: str) -> str:
        """Фраза кода из ``finished`` — исходной речью, не клипом. Иначе «Сказал.» сам станет звуком."""
        self.spoken.append(phrase)
        return self._say(turn_id, epoch, phrase)


def phrase_from_event(clip: Any, msg: Any) -> Optional[str]:
    """Фраза кода из события звука, если это клип режима и исход — ``finished`` без причины. Иначе ``None``."""
    return clip.gate.on_finished(
        str(getattr(msg, "request_id", "") or ""), int(getattr(msg, "kind", -1)), str(getattr(msg, "reason", "") or "")
    )


def aborted(stopped: Any, generation: int) -> bool:
    """Поток клипа оборван, если «стоп» случился после старта этого проигрывания.

    «Стоп» до входа в поток поколение не сбрасывает: он запоминается и виден здесь же.
    """
    return bool(getattr(stopped, "is_set", lambda: False)()) and int(generation) <= _generation(stopped)


def arm(stopped: Any) -> int:
    """Поколение на старте проигрывания. «Стоп» до входа в поток не забывается.

    Раньше флаг здесь снимался: STOP, пришедший между запросом и стартом потока, клип
    доигрывал и подтверждался. Теперь ранний стоп остаётся стопом.
    """
    return _generation(stopped)


def clip_outcome(stopped: Any, generation: Optional[int], played: bool) -> Optional[str]:
    """Исход клипа режима: ``stopped`` / ``finished`` / ``None`` (это не клип режима).

    ``generation is None`` — обычный эффект: «стоп» рации его исход не перекрашивает,
    решает то, что вернул ``play_audio``. Клип режима с поднятым флагом — ``stopped``,
    даже если устройство отчиталось об успешном конце (``sd.stop()`` гасит и естественный).
    """
    if generation is None:
        return None
    if aborted(stopped, generation):
        return "stopped"
    return "finished" if played else None


def _generation(stopped: Any) -> int:
    return int(getattr(stopped, "generation", 0))


def _session_generation(snapshot: Any) -> int:
    """Поколение сессии снимка. Снимок без поля (тест, старый кадр) — одна сессия на всё время."""
    return int(getattr(snapshot, "session_generation", 0))
