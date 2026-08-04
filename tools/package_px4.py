#!/usr/bin/env python3
"""Create a PX4 bootloader-compatible PX4FWv1 JSON firmware package."""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import zlib


def git_identity(repo: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "--verify", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prototype", required=True, type=Path)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    metadata = json.loads(args.prototype.read_text(encoding="utf-8"))
    if metadata.get("magic") != "PX4FWv1":
        raise SystemExit("prototype magic must be PX4FWv1")

    image = args.image.read_bytes()
    max_size = int(metadata["image_maxsize"])
    if len(image) > max_size:
        raise SystemExit(
            f"firmware is {len(image)} bytes; FMUv6C maximum is {max_size} bytes"
        )

    # PX4 bootloaders expect a word-aligned, 0xff-padded application image.
    padded = image + b"\xff" * ((-len(image)) % 4)
    metadata.update(
        build_time=int(dt.datetime.now(dt.timezone.utc).timestamp()),
        image_size=len(padded),
        image=base64.b64encode(zlib.compress(padded, 9)).decode("ascii"),
        image_sha256=hashlib.sha256(padded).hexdigest(),
        git_identity=git_identity(args.prototype.resolve().parents[2]),
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())