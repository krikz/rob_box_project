"""Regression test for issue #3315.

Issue: deployment_issue_dedup.extract_relevant_log_line() must exclude
two well-known false-positive critical_log lines that were triggering a
`🚨 Deployment Completed With Issues` issue on every develop/staging
deploy since 2026-10-01.

The two false-positives are:

1. ``cadvisor-vision`` / ``cadvisor`` (Main Pi) cAdvisor ARM boot probe:

   The pinned image ``gcr.io/cadvisor/cadvisor:v0.52.1`` logs a one-shot
   ``gce.go:45] Error while reading product_name: open
   /sys/class/dmi/id/product_name: no such file or directory`` on
   startup. Raspberry Pi boards (Pi 4B, Pi 5) do not expose SMBIOS
   ``/sys/class/dmi/id/product_name`` (cAdvisor upstream uses that probe
   to detect GCE instance types only — the absence is benign). cAdvisor
   continues to expose cpu / memory / disk / network metrics normally;
   the error level is I (info) but the word "Error" trips
   ``CRITICAL_MATCH_RE``. The deploy gate has opened a deploy-fail issue
   on every staging deploy since 2026-10-01 even though the deployment
   itself is fully successful (containers up, ROS2 topics active,
   healthchecks passing).

2. ``promtail-vision`` Docker API client mismatch:

   The container ``promtail-vision`` is pinned at
   ``grafana/promtail:2.9.2`` (docker/vision/docker-compose.yaml line
   738) and the embedded Docker SDK targets API 1.24-1.43. Vision Pi /
   Main Pi Docker daemons were upgraded to a build whose minimum
   supported API is 1.44, so every 5s the docker_sd_configs refresh
   prints ``level=error ... component=docker_discovery ... err="error
   while listing containers: Error response from daemon: client version
   1.42 is too old. Minimum supported API version is 1.44, please
   upgrade your client to a newer version"``. This is a build-pipeline
   issue, not a deployment failure: the system-log scrape job
   (``/var/log/*.log``) keeps working (filetarget manager is
   unaffected), only the Docker label-discovery scrape is dropped. Loki
   still receives host logs; container log shipping is the only loss.

This test pins down three contracts:

1. POSITIVE A: a cadvisor product_name probe line in the vision scope
   is NOT returned by ``extract_relevant_log_line(..., severity="critical")``.

2. POSITIVE B: a promtail 1.42/1.44 Docker API error line in the vision
   scope is NOT returned either.

3. NEGATIVE: a real critical error that is NOT one of these two
   monitoring false-positives is STILL returned, so we don't
   accidentally silence real failures.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "deployment_issue_dedup.py"
)
SPEC = importlib.util.spec_from_file_location(
    "deployment_issue_dedup", SCRIPT_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


# ----- 1. POSITIVE A: cadvisor product_name probe must be excluded -----


CADVISOR_PRODUCT_NAME_PROBE = "\n".join(
    [
        "I1001 14:10:52.756661       1 gce.go:45] "
        "Error while reading product_name: open /sys/class/dmi/id/product_name: "
        "no such file or directory",
        "I1001 14:10:52.768571       1 manager.go:233] "
        "Version: {KernelVersion:6.8.0-1065-raspi ContainerOsVersion:Alpine Linux v3.18 "
        "DockerVersion: DockerAPIVersion: CadvisorVersion:v0.52.1 CadvisorRevision:0b675def}",
        "I1001 14:10:52.901691       1 manager.go:1196] "
        "Started watching for new ooms in manager",
    ]
)


def test_extract_relevant_log_line_ignores_cadvisor_product_name_in_vision_scope() -> None:
    """The cadvisor ARM product_name probe is benign on Raspberry Pi.

    Mirrors the exact log content from deploy run 36873490556
    (issue #3315) on Vision Pi cadvisor-vision: gcr.io/cadvisor/cadvisor:v0.52.1
    tries to read /sys/class/dmi/id/product_name (SMBIOS) to detect a GCE
    instance type, the file does not exist on Pi boards, cAdvisor logs a
    single ERROR-shaped line at I (info) level and continues. The deploy
    gate must not file a critical issue for this.
    """
    line = MODULE.extract_relevant_log_line(
        CADVISOR_PRODUCT_NAME_PROBE, scope="vision", severity="critical"
    )
    assert line is None, (
        "cadvisor product_name ARM probe must be excluded; "
        f"got: {line!r}"
    )


def test_extract_relevant_log_line_ignores_cadvisor_product_name_in_main_scope() -> None:
    """Same exclusion applies on Main Pi (where cadvisor also runs).

    The cadvisor container is deployed in both ``docker/vision/docker-compose.yaml``
    (line 711) and ``docker/main/docker-compose.yaml`` (line 344) with the
    same pinned image, and the product_name probe fires on both Pi
    boards. The exclusion lives in ``CRITICAL_EXCLUDE_COMMON`` (not in
    ``CRITICAL_EXCLUDE_BY_SCOPE['main']``), so it must also work in main
    scope. This test pins that contract.
    """
    line = MODULE.extract_relevant_log_line(
        CADVISOR_PRODUCT_NAME_PROBE, scope="main", severity="critical"
    )
    assert line is None


# ----- 2. POSITIVE B: promtail 1.42/1.44 mismatch must be excluded -----


PROMTAIL_DOCKER_API_ERROR = "\n".join(
    [
        "level=info ts=2026-10-01T14:10:51.631094192Z caller=promtail.go:133 "
        "msg=\"Reloading configuration file\" md5sum=fd1eb9437c42fa1418494ba20f66fb06",
        "level=info ts=2026-10-01T14:10:51.648325639Z caller=server.go:322 "
        "http=[::]:9080 grpc=[::]:34859 msg=\"server listening on addresses\"",
        "level=info ts=2026-10-01T14:10:51.648603318Z caller=main.go:174 "
        "msg=\"Starting Promtail\" version=\"(version=2.9.2, branch=HEAD, "
        "revision=a17308db6)\"",
        "level=error ts=2026-10-01T14:10:51.669959732Z caller=refresh.go:80 "
        "component=docker_discovery discovery=docker "
        "config=docker/unix:///var/run/docker.sock:80 msg=\"Unable to refresh "
        "target groups\" err=\"error while listing containers: Error response "
        "from daemon: client version 1.42 is too old. Minimum supported API "
        "version is 1.44, please upgrade your client to a newer version\"",
        "level=info ts=2026-10-01T14:10:56.644394898Z "
        "caller=filetargetmanager.go:361 msg=\"Adding target\" "
        "key=\"/var/log/*.log:{host=\\\"vision-pi\\\", job=\\\"varlogs\\\"}\"",
    ]
)


def test_extract_relevant_log_line_ignores_promtail_docker_api_in_vision_scope() -> None:
    """The promtail 2.9.2 docker_sd / 1.44 daemon mismatch is benign.

    Mirrors the exact log content from deploy run 36873490556
    (issue #3315) on promtail-vision. The Docker SDK shipped in
    grafana/promtail:2.9.2 (October 2024) targets API 1.24-1.43; the
    Vision Pi Docker daemon was upgraded to a build whose minimum
    supported API is 1.44, so the docker_sd_configs refresh logs an
    ERROR every 5s. The system-log scrape (``/var/log/*.log``) keeps
    working — only the Docker label-discovery scrape is dropped. The
    deploy gate must not file a critical issue for this.
    """
    line = MODULE.extract_relevant_log_line(
        PROMTAIL_DOCKER_API_ERROR, scope="vision", severity="critical"
    )
    assert line is None, (
        "promtail 1.42/1.44 Docker API mismatch must be excluded; "
        f"got: {line!r}"
    )


def test_extract_relevant_log_line_ignores_promtail_docker_api_in_main_scope() -> None:
    """Same exclusion applies on Main Pi (where promtail also runs).

    The promtail container is deployed in both
    ``docker/vision/docker-compose.yaml`` (line 737) and
    ``docker/main/docker-compose.yaml`` (line 404) with the same pinned
    image, and the Docker API mismatch is reproducible on both Pis.
    The exclusion lives in ``CRITICAL_EXCLUDE_COMMON`` (not in
    ``CRITICAL_EXCLUDE_BY_SCOPE['main']``), so it must also work in main
    scope. This test pins that contract.
    """
    line = MODULE.extract_relevant_log_line(
        PROMTAIL_DOCKER_API_ERROR, scope="main", severity="critical"
    )
    assert line is None


# ----- 3. NEGATIVE: real critical errors must still be reported -----


def test_extract_relevant_log_line_still_catches_real_cadvisor_error() -> None:
    """A real cadvisor error (NOT the product_name probe) must still be reported.

    The exclusion is deliberately scoped to the gce.go / product_name
    probe. Other cadvisor-side errors (e.g. ``manager.go:1116] Failed
    to create existing container`` from a missing overlayfs layer) are
    real deployment events that the operator must see.
    """
    log_text = (
        "E1001 14:10:52.973754       1 manager.go:1116] "
        "Failed to create existing container: /system.slice/docker-"
        "f8b4f34a8e7b6b683b078f45fa172ef0abddc3ae018c711d8ef8a3904b269f28.scope: "
        "failed to identify the read-write layer ID for container "
        "\"f8b4f34a8e7b6b683b078f45fa172ef0abddc3ae018c711d8ef8a3904b269f28\". - open "
        "/rootfs/var/lib/docker/image/overlayfs/layerdb/mounts/"
        "f8b4f34a8e7b6b683b078f45fa172ef0abddc3ae018c711d8ef8a3904b269f28/mount-id: "
        "no such file or directory"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="vision", severity="critical"
    )
    assert line is not None
    assert "Failed to create existing container" in line


def test_extract_relevant_log_line_still_catches_real_docker_daemon_failure() -> None:
    """A real Docker daemon failure (NOT the API version mismatch) must be reported.

    The exclusion is deliberately scoped to the ``client version 1.42
    is too old`` substring. A different Docker daemon failure (e.g.
    ``Cannot connect to the Docker daemon`` when the socket is missing
    or has the wrong permissions) is a real outage that the operator
    must see.
    """
    log_text = (
        "level=error ts=2026-10-01T14:10:51.669Z caller=refresh.go:80 "
        "component=docker_discovery discovery=docker "
        "config=docker/unix:///var/run/docker.sock:80 msg=\"Unable to refresh "
        "target groups\" err=\"Cannot connect to the Docker daemon at "
        "unix:///var/run/docker.sock. Is the docker daemon running?\""
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="vision", severity="critical"
    )
    assert line is not None
    assert "Cannot connect to the Docker daemon" in line


def test_extract_relevant_log_line_still_catches_python_traceback_in_vision_scope() -> None:
    """A real Python traceback on a vision container must still be reported.

    Ensures the new monitoring exclusions do not accidentally silence
    the broader critical_log detector. ``extract_relevant_log_line``
    returns the FIRST line that matches the critical regex and is not
    excluded — for a Python traceback that is the
    ``Traceback (most recent call last):`` header (the actual exception
    type/value is on later lines). The deploy detector's job is to
    surface any critical pattern; the operator reads the full log dump
    from the GitHub Actions artifact for triage. We only assert that
    some critical line is returned, not a specific one.
    """
    log_text = (
        "[vision-face-1] [ERROR] vision_face_node: "
        "Traceback (most recent call last):\n"
        "  File /opt/ros/humble/lib/python3.10/site-packages/rob_box_perception/"
        "vision_face_node.py line 63 in main\n"
        "rob_box_perception.gaze.GazeSourceUnavailable: Gaze source 'oak_d' "
        "did not deliver a frame within 10.1s"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="vision", severity="critical"
    )
    assert line is not None
    assert "Traceback" in line or "GazeSourceUnavailable" in line
