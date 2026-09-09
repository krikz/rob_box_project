#!/usr/bin/env python3
"""Convert a Markdown file to PDF using Python markdown + Chrome headless."""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import markdown

CSS = """
@page { size: A4; margin: 20mm 20mm 20mm 20mm; }
body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif;
    font-size: 11pt;
    line-height: 1.6;
    color: #24292e;
    max-width: 100%;
}
h1 { font-size: 2em; border-bottom: 2px solid #e1e4e8; padding-bottom: 0.3em; margin-top: 1.2em; }
h2 { font-size: 1.5em; border-bottom: 1px solid #e1e4e8; padding-bottom: 0.2em; margin-top: 1em; }
h3 { font-size: 1.2em; margin-top: 0.8em; }
h4 { font-size: 1em; margin-top: 0.6em; }
code {
    background: #f6f8fa;
    border-radius: 3px;
    font-size: 0.9em;
    padding: 0.2em 0.4em;
    font-family: "SFMono-Regular", Consolas, "Liberation Mono", Menlo, monospace;
}
pre {
    background: #f6f8fa;
    border-radius: 6px;
    padding: 16px;
    overflow: auto;
    font-size: 0.85em;
    line-height: 1.45;
    page-break-inside: avoid;
}
pre code { background: transparent; padding: 0; }
table {
    border-collapse: collapse;
    width: 100%;
    margin: 1em 0;
    page-break-inside: avoid;
    font-size: 0.9em;
}
th, td { border: 1px solid #dfe2e5; padding: 6px 13px; }
th { background: #f6f8fa; font-weight: 600; }
tr:nth-child(even) { background: #f6f8fa; }
blockquote {
    border-left: 4px solid #dfe2e5;
    color: #6a737d;
    margin: 0;
    padding: 0 1em;
}
a { color: #0366d6; text-decoration: none; }
hr { border: none; border-top: 1px solid #e1e4e8; margin: 1.5em 0; }
ul, ol { padding-left: 2em; }
li { margin: 0.2em 0; }
"""

CHROME_PATHS = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
]


def find_chrome() -> str | None:
    for path in CHROME_PATHS:
        if Path(path).exists():
            return path
    return None


def convert(md_path: Path, pdf_path: Path) -> None:
    md_text = md_path.read_text(encoding="utf-8")

    html_body = markdown.markdown(
        md_text,
        extensions=["tables", "fenced_code", "toc", "nl2br", "sane_lists"],
    )
    html = f"""<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<style>{CSS}</style>
</head>
<body>
{html_body}
</body>
</html>"""

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".html", delete=False, encoding="utf-8"
    ) as f:
        f.write(html)
        html_file = Path(f.name)

    chrome = find_chrome()
    if not chrome:
        print("ERROR: Chrome not found", file=sys.stderr)
        sys.exit(1)

    try:
        result = subprocess.run(
            [
                chrome,
                "--headless=new",
                "--disable-gpu",
                "--no-sandbox",
                "--disable-extensions",
                f"--print-to-pdf={pdf_path.resolve()}",
                "--print-to-pdf-no-header",
                html_file.as_uri(),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        if result.returncode != 0:
            print(f"Chrome stderr: {result.stderr}", file=sys.stderr)
            sys.exit(result.returncode)
    finally:
        html_file.unlink(missing_ok=True)

    print(f"✓ PDF сохранён: {pdf_path.resolve()}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert Markdown to PDF via Chrome")
    parser.add_argument("input", type=Path, help="Input .md file")
    parser.add_argument("--output", "-o", type=Path, help="Output .pdf file (default: same name)")
    args = parser.parse_args()

    md_path = args.input.resolve()
    pdf_path = args.output.resolve() if args.output else md_path.with_suffix(".pdf")

    if not md_path.exists():
        print(f"ERROR: file not found: {md_path}", file=sys.stderr)
        sys.exit(1)

    convert(md_path, pdf_path)


if __name__ == "__main__":
    main()
