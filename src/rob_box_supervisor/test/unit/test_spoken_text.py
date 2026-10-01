"""Тесты речевой версии ответа ТАРС (issue #3296)."""

from __future__ import annotations

import unittest

from rob_box_supervisor.spoken_text import DETAILS_SUFFIX, to_spoken

ISSUE_ANSWER = (
    "Доступные запросы к Prometheus (PromQL) и Loki (LogQL):\n\n"
    "**Prometheus — процесс/система:**\n"
    "- `cpu` — общий CPU процесса\n"
    "- `memory` / `process_resident_memory_bytes` — RSS по PID\n"
    "- `up` — статус экспортёра\n"
    "- `rate(process_cpu_seconds_total[5m])` — загрузка\n\n"
    "**Latency / ошибки:**\n"
    "- `rate(voice_llm_request_duration_seconds_sum[5m]) / "
    "rate(voice_llm_request_duration_seconds_count[5m])` — задержка\n"
)


class TestToSpoken(unittest.TestCase):
    def test_issue_answer_is_short_and_clean(self) -> None:
        out = to_spoken(ISSUE_ANSWER)
        self.assertLessEqual(len(out), 200 + len(DETAILS_SUFFIX) + 1)
        for bad in ("`", "*", "#", "rate(", "_seconds", "[5m]"):
            self.assertNotIn(bad, out)
        self.assertTrue(out.endswith(DETAILS_SUFFIX))

    def test_short_reply_unchanged(self) -> None:
        self.assertEqual(to_spoken("Ок"), "Ок")
        self.assertEqual(to_spoken("Готово. Камера повёрнута."), "Готово. Камера повёрнута.")

    def test_empty(self) -> None:
        self.assertEqual(to_spoken(""), "")
        self.assertEqual(to_spoken("   \n "), "")

    def test_only_code_is_empty(self) -> None:
        self.assertEqual(to_spoken("```\nrate(x[5m])\n```"), "")

    def test_long_paragraph_cut_on_sentence_boundary(self) -> None:
        text = "Первая фраза номер один. " + "Вторая фраза " * 20 + "конец. Третья."
        out = to_spoken(text, max_chars=60, details_suffix="")
        self.assertEqual(out, "Первая фраза номер один.")

    def test_single_giant_sentence_cut_on_word(self) -> None:
        out = to_spoken("слово " * 100, max_chars=30, details_suffix="")
        self.assertLessEqual(len(out), 31)
        self.assertTrue(out.endswith("…"))

    def test_url_and_link_removed(self) -> None:
        out = to_spoken("Смотри [документ](http://x.y/z) тут http://a.b/c.")
        self.assertNotIn("http", out)
        self.assertIn("документ", out)

    def test_at_most_two_sentences(self) -> None:
        out = to_spoken("Раз. Два. Три. Четыре.", details_suffix="")
        self.assertEqual(out, "Раз. Два.")


if __name__ == "__main__":
    unittest.main()
