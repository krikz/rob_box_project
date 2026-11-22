"""#5039: паника zenoh в nav2 должна печатать стек, а не только место unwrap.

22.11 18:47:20Z controller_server упал на resource.rs:764 и сам написал
«run with RUST_BACKTRACE=1». Переменная живёт в environment сервиса nav2;
ros_with_namespace.sh и start_nav2_direct.sh её не сбрасывают.
"""

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
COMPOSE = REPO / "docker" / "main" / "docker-compose.yaml"


def test_nav2_enables_rust_backtrace():
    compose = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))
    environment = compose["services"]["nav2"]["environment"]
    assert "RUST_BACKTRACE=1" in environment
    assert "RUST_LOG=zenoh=info" in environment
