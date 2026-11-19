"""Внешние образы релиза должны попадать в compose по digest, а не плавающим тегом.

Релиз закрепляет eclipse/zenoh и прочие external_images по digest, и adopt
отказывает, если они не закреплены. Но stage подставляет digest только в
переменные *_TAG наших сервисов: сторонний образ в compose остаётся тегом.
Звено 1 (#4375) требует, чтобы итоговый compose config для каждой роли
использовал подписанные digest и для них.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from chuangong.stage import compose_digest_env, external_override


def test_external_override_pins_every_external_image(tmp_path: Path) -> None:
    release = {
        "image_tag": "dev-test",
        "roles": {
            "main": {
                "images": {"nav2": "reg/rob_box@sha256:" + "a" * 64},
                "external_images": [
                    "eclipse/zenoh:1.6.2@sha256:" + "b" * 64,
                    "grafana/promtail:3.6.11@sha256:" + "c" * 64,
                ],
            }
        },
    }
    text = external_override(release, "main")
    doc = yaml.safe_load(text)
    images = {svc["image"] for svc in doc["services"].values()}
    assert set(release["roles"]["main"]["external_images"]) <= images
    assert all("@sha256:" in img for img in images)


def test_compose_env_alone_leaves_external_images_unpinned() -> None:
    """Без override env stage закрепляет только наши сервисы — это и есть пробел."""
    env = compose_digest_env(
        {"nav2": "reg/rob_box@sha256:" + "a" * 64},
        image_tag="dev-test",
        service_vars={"nav2": "NAV2_TAG"},
    )
    assert "eclipse/zenoh:1.6.2" not in env.values()
