"""One-shot CLI wrapper around :func:`upsert_summary_comment`.

Usage
-----

    # First publish (creates a new summary comment):
    python -m pr907.publish_summary --body-file analysis/pr-907-final-summary-comment.md

    # Re-publish (edits the existing summary in place):
    python -m pr907.publish_summary \\
        --body-file analysis/pr-907-final-summary-comment.md \\
        --existing-id 1234567890

    # Or pipe the body on stdin:
    cat analysis/pr-907-final-summary-comment.md \\
        | python -m pr907.publish_summary --existing-id 1234567890

The script reads the GitHub token from ``$GITHUB_TOKEN`` and exits with
a non-zero status on any error, printing the structured envelope as JSON
on success so the caller can scrape the resulting ``comment_id`` /
``comment_url``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from pr907.github_summary_comment import upsert_summary_comment


def _load_body(args: argparse.Namespace) -> str:
    """Read the comment body from ``--body-file`` or stdin (one of the two)."""
    if args.body_file is not None:
        return args.body_file.read_text(encoding="utf-8")

    if not sys.stdin.isatty():
        return sys.stdin.read()

    print(
        "ERROR: provide --body-file PATH or pipe the body on stdin",
        file=sys.stderr,
    )
    raise SystemExit(2)


def _parse_existing_id(raw: str | None) -> int | None:
    if raw is None:
        return None
    try:
        value = int(raw)
    except ValueError as exc:
        raise SystemExit(f"--existing-id must be an integer, got: {raw!r}") from exc
    if value <= 0:
        raise SystemExit(f"--existing-id must be > 0, got: {value}")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--body-file", type=Path, default=None,
        help="Path to the markdown body file. Mutually exclusive with stdin.",
    )
    parser.add_argument(
        "--existing-id", type=str, default=None,
        help="If set, PATCH the comment with this id; otherwise POST a new one.",
    )
    parser.add_argument(
        "--no-issue-number", action="store_true",
        help="Omit the optional 'issue_number' field from the POST body.",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print the request that would be issued without contacting GitHub.",
    )
    args = parser.parse_args(argv)

    if "GITHUB_TOKEN" not in os.environ and not args.dry_run:
        print(
            "ERROR: $GITHUB_TOKEN is not set. Export it or pass --dry-run.",
            file=sys.stderr,
        )
        return 2

    body = _load_body(args)
    existing_id = _parse_existing_id(args.existing_id)

    if args.dry_run:
        verb = "PATCH" if existing_id else "POST"
        url = (
            f"https://api.github.com/repos/krikz/rob_box_project/issues/comments"
            f"/{existing_id}"
            if existing_id
            else "https://api.github.com/repos/krikz/rob_box_project/issues/comments"
        )
        payload: dict[str, object] = {"body": body}
        if verb == "POST" and not args.no_issue_number:
            payload["issue_number"] = 907
        envelope = {
            "dry_run": True,
            "verb": verb,
            "url": url,
            "payload": payload,
            "body_bytes": len(body.encode("utf-8")),
        }
        print(json.dumps(envelope, ensure_ascii=False, indent=2))
        return 0

    try:
        result = upsert_summary_comment(
            body,
            existing_id,
            include_issue_number_in_create=not args.no_issue_number,
        )
    except Exception as exc:  # noqa: BLE001 — surface all errors uniformly
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())