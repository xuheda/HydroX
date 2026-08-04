"""Minimal binary compatibility test for the XLog dataset reader."""
from __future__ import annotations

import json
import struct
import tempfile
import zlib
from pathlib import Path

from xlog_dataset import (BLOCK_HEADER, FILE_HEADER, RECORD_HEADER, TOPIC_ACTUATOR,
                          TOPIC_CONTROLLER_OUTPUT, TOPIC_STATE, build_dataset)


def record(topic: int, timestamp_ns: int, sequence: int, payload: bytes) -> bytes:
    return RECORD_HEADER.pack(topic, 0, len(payload), timestamp_ns, sequence) + payload


def state(nu_u: float) -> bytes:
    return struct.pack("<13d8B", *(0.0 for _ in range(6)), nu_u, 0.0, 0.0, 0.0, 0.0, 0.0,
                       2.0, 1, 0, 1, 1, 0, 0, 0, 0)


def main() -> None:
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        path = directory / "fixture.xlog"
        metadata = json.dumps({"vehicle": "fixture"}).encode()
        schema = b"{}"
        payload = b"".join((
            record(TOPIC_CONTROLLER_OUTPUT, 1_000_000_000, 1, struct.pack("<7d", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0)),
            record(TOPIC_ACTUATOR, 1_000_000_000, 2, struct.pack("<8f4d8B", *(0.1 for _ in range(8)), 100.0, 0.0, 0.1, 0.0, 1, 0, 0, 0, 0, 0, 0, 0)),
            record(TOPIC_STATE, 1_000_000_000, 3, state(1.0)),
            record(TOPIC_CONTROLLER_OUTPUT, 1_010_000_000, 4, struct.pack("<7d", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0)),
            record(TOPIC_ACTUATOR, 1_010_000_000, 5, struct.pack("<8f4d8B", *(0.1 for _ in range(8)), 100.0, 0.0, 0.1, 0.0, 1, 0, 0, 0, 0, 0, 0, 0)),
            record(TOPIC_STATE, 1_010_000_000, 6, state(1.1)),
        ))
        header = FILE_HEADER.pack(b"XLOG", 1, 0, FILE_HEADER.size, 1, 0, RECORD_HEADER.size,
                                  BLOCK_HEADER.size, 0, 0, 0, 0, len(metadata), len(schema), 0,
                                  0, 0, 0, 0)
        block = BLOCK_HEADER.pack(b"XBLK", BLOCK_HEADER.size, 1, 0, len(payload), 6,
                                  zlib.crc32(payload) & 0xFFFFFFFF, 1_000_000_000,
                                  1_010_000_000, 1, 6, 0, 0)
        path.write_bytes(header + metadata + schema + block + payload)
        output = directory / "dataset.npz"
        manifest = build_dataset([path], output)
        if manifest["samples"] != 1 or not output.exists():
            raise AssertionError("XLog fixture did not produce one transition")
    print("test_xlog_dataset: PASS")


if __name__ == "__main__":
    main()
