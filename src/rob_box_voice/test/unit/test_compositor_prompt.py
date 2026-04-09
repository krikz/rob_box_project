from pathlib import Path


def test_compositor_prompt_does_not_embed_event_only_routing() -> None:
    prompt_path = (
        Path(__file__).resolve().parents[2]
        / "prompts"
        / "compositor_prompt.txt"
    )

    content = prompt_path.read_text(encoding="utf-8")

    assert "STYLE OR PERFORMANCE DOES NOT CANCEL EVENT FAQ" not in content
    assert "зачитай рэп про коррупцию" not in content