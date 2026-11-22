"""#5103: явно названный голос из каталога провайдера не выбрасывается.

Живой ход 8fe6bfac2e66: форма ``voice`` открыта (реестр имён), модель прислала ``English_Jovialman``,
валидатор сверял его с каталогом РОЛЕЙ (``Busy.voices`` = ``role_voices_for``) и клал ``voice`` в ``dropped``.
TTS брал ``default``. Решение Шифу 22.11.2026: голос из каталога провайдера — используем; чужое имя — по-прежнему
``dropped``. Каталог ролей не расширяется (#3998, #4084).
"""

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog.command import Busy, Command, Form, parse, validate
from rob_box_dialog_host.tts_voice_registry import PROVIDER_VOICES, named_voices_for, role_voices_for, voices_for

ALL = [t.name for t in TOOL_CATALOG]
ASKED = Form(passed=False, voice=True)  # человек назвал голос: форма открыла voice
JOVIALMAN = "English_Jovialman"


def _roles(provider: str = "minimax") -> tuple[str, ...]:
    """Каталог ролей, как его кладёт планировщик: русские голоса без рассказчика."""
    return tuple(role_voices_for(provider, "Russian_ReliableMan"))


def _busy(provider: str = "minimax", *, catalogue: bool = True) -> Busy:
    voices = _roles(provider)
    return Busy(
        voices=voices,
        voice_catalogue=tuple(voices_for(provider)) if catalogue else (),
    )


def test_named_voice_from_the_provider_catalogue_is_kept():
    """(а) ``English_Jovialman`` есть у minimax и не входит в роли: остаётся в ``Command.voice``."""
    busy = _busy()
    assert JOVIALMAN not in busy.voices and JOVIALMAN in busy.voice_catalogue
    res = validate(parse({"speech": "А вот и я.", "voice": JOVIALMAN}), ALL, busy=busy, form=ASKED)
    assert isinstance(res, Command)
    assert res.voice == JOVIALMAN and "voice" not in res.dropped


def test_voice_absent_from_the_catalogue_is_dropped():
    """(б) Чужое имя в TTS не уходит: поле в ``dropped``, речь остаётся."""
    res = validate(parse({"speech": "Привет", "voice": "not-a-voice"}), ALL, busy=_busy(), form=ASKED)
    assert isinstance(res, Command)
    assert res.voice is None and res.dropped == ("voice",) and res.speech == "Привет"


def test_role_catalogue_does_not_grow_to_fit_the_named_voice():
    """(в) ``Busy.voices`` — по-прежнему роли: ``role_voices_for`` без ``English_*``."""
    roles = _roles()
    assert roles and all(not voice.startswith("English_") for voice in roles)
    assert JOVIALMAN not in role_voices_for("minimax")
    kept = validate(parse({"speech": "Привет", "voice": JOVIALMAN}), ALL, busy=_busy(), form=ASKED)
    assert kept.voice == JOVIALMAN
    assert _busy().role_pool() == tuple((voice, "") for voice in roles)


def test_empty_catalogue_still_checks_the_role_list():
    """Каталог провайдера пуст (старый вызов, перепроверка вне хода) — сверка по ролям, как раньше."""
    busy = _busy(catalogue=False)
    foreign = validate(parse({"speech": "Привет", "voice": JOVIALMAN}), ALL, busy=busy, form=ASKED)
    assert foreign.voice is None and "voice" in foreign.dropped
    role = busy.voices[0]
    assert validate(parse({"speech": "Привет", "voice": role}), ALL, busy=busy, form=ASKED).voice == role


def test_unasked_voice_is_still_dropped():
    """Форма закрыла ``voice`` (голоса не просили) — поле лишнее, даже если имя есть в каталоге."""
    closed = Form(passed=False, voice=False)
    res = validate(parse({"speech": "Привет", "voice": JOVIALMAN}), ALL, busy=_busy(), form=closed)
    assert res.voice is None and "voice" in res.dropped


def test_every_voice_the_registry_shows_the_model_is_accepted():
    """(г) Что реестр показывает модели (``voices_for`` / ``named_voices_for``) ⊆ что принимает валидатор.

    Цикл по провайдерам, не один кейс: enum схемы и сверка читают один каталог (``voice_catalogue``).
    """
    for provider in PROVIDER_VOICES:
        shown = voices_for(provider)
        assert shown, provider
        assert {entry[0] for entry in named_voices_for(provider)} == set(shown)
        busy = Busy(voices=tuple(role_voices_for(provider)), voice_catalogue=tuple(shown))
        assert set(busy.line_voices()) == set(shown)
        for voice in shown:
            res = validate(parse({"speech": "Привет", "voice": voice}), ALL, busy=busy, form=ASKED)
            assert isinstance(res, Command) and res.voice == voice and "voice" not in res.dropped, (
                provider,
                voice,
                res.dropped if isinstance(res, Command) else res,
            )


def test_bridge_schema_enum_is_the_provider_catalogue_not_the_roles():
    """Мост показывает модели тот же каталог, что принимает валидатор: не роли.

    Без этого enum схемы режет ``English_Jovialman`` ещё до валидатора (#5103).
    """
    from rob_box_dialog import knowledge as kn
    from rob_box_dialog.command import form_schema
    from rob_box_dialog_host.command_bridge import CommandModeLLM
    from rob_box_llm.provider import LLMProvider, ProviderCapabilities

    class _Inner(LLMProvider):
        name = "fake"

        @property
        def capabilities(self) -> ProviderCapabilities:
            return ProviderCapabilities(tools=True, tool_choice=True)

        async def complete(self, messages, *, tools=(), settings=None):
            raise NotImplementedError

        async def stream(self, messages, *, tools=(), settings=None):
            raise NotImplementedError
            yield  # pragma: no cover — абстрактный поток, shape его не зовёт

    busy = _busy()
    wrapped = CommandModeLLM(_Inner(), busy=lambda: busy)
    _messages, tools, _settings = wrapped.shape([], (), None)
    enum = next(t for t in tools if (t.get("function") or {}).get("name") == "command")
    names = enum["function"]["parameters"]["properties"]["voice"]["enum"]
    assert JOVIALMAN in names and set(names) == {"", *busy.voice_catalogue}
    assert set(busy.voices) < set(names), "роли — строгое подмножество каталога, не сам enum"
    assert (
        names
        == form_schema(kn.with_voices(kn.command_schema("json"), busy.line_voices()))["properties"]["voice"]["enum"]
    )
