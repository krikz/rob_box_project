from pathlib import Path


def test_voice_base_requirements_include_openpyxl_for_xlsx_faq_support() -> None:
    requirements_path = (
        Path(__file__).resolve().parents[4] / "docker" / "vision" / "voice_base" / "requirements.txt"
    )

    requirements = requirements_path.read_text(encoding="utf-8")

    assert "openpyxl" in requirements