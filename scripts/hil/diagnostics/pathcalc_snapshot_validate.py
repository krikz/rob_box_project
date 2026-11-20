#!/usr/bin/env python3
import struct
import sys
from pathlib import Path

HEADER = struct.Struct("<8sIii9d4iBBq4iiffi")
VERSION = 1


def validate(path: Path) -> None:
    actual_size = path.stat().st_size
    with path.open("rb") as stream:
        header = stream.read(HEADER.size)
    if len(header) != HEADER.size:
        raise ValueError(f"truncated header: {len(header)} != {HEADER.size}")
    values = HEADER.unpack(header)
    if values[0] != b"NAVFSNAP":
        raise ValueError("bad magic")
    if values[1] != VERSION:
        raise ValueError(f"unsupported version: {values[1]}")
    nx, ny = values[2:4]
    npath = values[27]
    if nx <= 0 or ny <= 0 or npath < 0:
        raise ValueError(f"invalid dimensions: nx={nx} ny={ny} npath={npath}")
    expected_size = HEADER.size + 5 * nx * ny + 8 * npath
    if actual_size != expected_size:
        raise ValueError(
            f"snapshot length: actual={actual_size} expected={expected_size}"
        )
    print(
        f"PATHCALC_SNAPSHOT_OK version={VERSION} nx={nx} ny={ny} "
        f"npath={npath} bytes={actual_size}"
    )


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} SNAPSHOT", file=sys.stderr)
        return 2
    try:
        validate(Path(sys.argv[1]))
    except (OSError, ValueError, struct.error) as exc:
        print(f"PATHCALC_SNAPSHOT_INVALID: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
