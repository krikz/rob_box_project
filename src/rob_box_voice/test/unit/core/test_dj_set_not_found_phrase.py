"""Отказ сета «названное не нашлось» (#3493): роутер говорит фразу кода из результата ``dj_set``, а не общее
«не получилось», и не ждёт ``started``. Фраза — ``rob_box_music.dj_line.set_not_found_text``."""

from __future__ import annotations

import asyncio

from rob_box_music.dj_line import set_not_found_text
from rob_box_voice.core.media_plan_run import run_media_plan
from rob_box_voice.core.media_router import DJ_FAIL_TEXT, MediaState, MediaRouter


def _plan():
    return MediaRouter().route("включи диджей сет на тему лебединое озеро", MediaState())


def test_not_found_refusal_is_spoken_as_built_by_code():
    text = set_not_found_text(["лебединое озеро"])

    async def tool(_call):
        return False, text

    ok, phrase, done = asyncio.run(run_media_plan(_plan(), tool, None))
    assert not ok and phrase == text and done == []


def test_other_failures_keep_the_generic_phrase():
    async def tool(_call):
        return False, "сет не начался: not_started"

    assert asyncio.run(run_media_plan(_plan(), tool, None))[1] == DJ_FAIL_TEXT
