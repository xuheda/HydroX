#!/usr/bin/env python3
"""Validate and fingerprint a HydroX FMUv6C HITL firmware release candidate."""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import zlib


FLASH_ORIGIN = 0x08020000
FLASH_LENGTH = 1920 * 1024
BOARD_ID = 56
DEFAULT_PROFILE_ID = "generic-auv-fin"
EXPECTED_PROFILE_COUNT = 10
BASENAME = "hydrox_pixhawk6cmini_hitl"
MIN_HITL_TASK_STACK = 64 * 1024
MAX_STATIC_STACK_FRAME = 8 * 1024
EIGEN_DYNAMIC_STACK_LIMIT = 4 * 1024
MAX_EIGEN_DYNAMIC_STATIC_FRAME = 1024
PX4_BOOT_RTC_SIGNATURE = 0xB007B007


def fail(message: str) -> None:
    raise SystemExit(f"firmware verification failed: {message}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git(repo: Path, *args: str) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), *args],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def parse_ihex(path: Path) -> dict[int, int]:
    memory: dict[int, int] = {}
    upper = 0
    eof_seen = False
    for line_number, raw in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        if not line.startswith(":"):
            fail(f"invalid Intel HEX prefix at line {line_number}")
        try:
            record = bytes.fromhex(line[1:])
        except ValueError:
            fail(f"invalid Intel HEX data at line {line_number}")
        if len(record) < 5 or len(record) != record[0] + 5:
            fail(f"invalid Intel HEX length at line {line_number}")
        if sum(record) & 0xFF:
            fail(f"invalid Intel HEX checksum at line {line_number}")
        count = record[0]
        offset = (record[1] << 8) | record[2]
        record_type = record[3]
        payload = record[4 : 4 + count]
        if record_type == 0x00:
            for index, value in enumerate(payload):
                address = upper + offset + index
                if address in memory:
                    fail(f"overlapping Intel HEX data at 0x{address:08x}")
                memory[address] = value
        elif record_type == 0x01:
            eof_seen = True
        elif record_type == 0x02:
            if count != 2:
                fail(f"invalid segment-address record at line {line_number}")
            upper = int.from_bytes(payload, "big") << 4
        elif record_type == 0x04:
            if count != 2:
                fail(f"invalid linear-address record at line {line_number}")
            upper = int.from_bytes(payload, "big") << 16
        elif record_type not in (0x03, 0x05):
            fail(f"unsupported Intel HEX record type 0x{record_type:02x}")
    if not eof_seen or not memory:
        fail("Intel HEX has no EOF record or no image data")
    return memory


def enabled_config(path: Path) -> set[str]:
    result: set[str] = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.fullmatch(r"(CONFIG_[A-Z0-9_]+)=y", line.strip())
        if match:
            result.add(match.group(1))
    return result


def config_integer(path: Path, key: str) -> int:
    prefix = f"{key}="
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith(prefix):
            try:
                return int(line[len(prefix) :], 0)
            except ValueError:
                fail(f"{key} is not an integer")
    fail(f"required configuration value is absent: {key}")
    raise AssertionError("unreachable")


def verify_stack_usage(build: Path) -> tuple[int, str, int, int, int]:
    stack_files = sorted(
        (build / "apps" / "external" / "hydrox_hitl").rglob("*.su")
    )
    if not stack_files:
        fail("compiler stack-usage reports are missing")

    entries: list[tuple[int, str, str]] = []
    entry_frame = None
    for path in stack_files:
        for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
            fields = raw.rsplit("\t", 2)
            if len(fields) != 3:
                continue
            prefix, size_text, qualifier = fields
            try:
                size = int(size_text)
            except ValueError:
                continue
            match = re.match(r"^.*:\d+:\d+:(.*)$", prefix)
            function = match.group(1) if match else prefix
            entries.append((size, function, qualifier))
            if "hydrox_hitl_main" in function:
                entry_frame = size

    if not entries:
        fail("compiler stack-usage reports contain no parseable functions")
    if entry_frame is None:
        fail("compiler stack-usage report is missing hydrox_hitl_main")

    dynamic = [entry for entry in entries if "dynamic" in entry[2]]
    # Eigen's runtime stack helper uses alloca only below
    # EIGEN_STACK_ALLOCATION_LIMIT and otherwise falls back to the heap. The
    # application has a compile-time assertion for that definition. GCC still
    # labels those internal helpers "dynamic" because it cannot infer the
    # macro's runtime branch, so permit only the narrowly identified Eigen
    # internals and keep their compiler-reported fixed portion small.
    unexpected_dynamic = [
        entry
        for entry in dynamic
        if "bounded" not in entry[2]
        and not (
            "Eigen::internal::" in entry[1]
            and entry[0] <= MAX_EIGEN_DYNAMIC_STATIC_FRAME
        )
    ]
    if unexpected_dynamic:
        fail(f"unbounded application stack usage: {unexpected_dynamic[0][1]}")

    oversized = sorted(
        (entry for entry in entries if entry[0] > MAX_STATIC_STACK_FRAME),
        reverse=True,
    )
    if oversized:
        size, function, qualifier = oversized[0]
        fail(
            f"static stack frame {size} exceeds {MAX_STATIC_STACK_FRAME} bytes: "
            f"{function} ({qualifier})"
        )

    size, function, _ = max(entries)
    max_dynamic_static_frame = max((entry[0] for entry in dynamic), default=0)
    return size, function, entry_frame, len(dynamic), max_dynamic_static_frame


def require_file(path: Path) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        fail(f"required output is missing or empty: {path}")


def fnv1a64(data: bytes) -> int:
    value = 14695981039346656037
    for byte in data:
        value ^= byte
        value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value


def load_profile_registry(repo: Path) -> tuple[Path, dict, list[dict]]:
    generator = repo / "tools" / "generate_fmuv6c_profile_registry.py"
    require_file(generator)
    result = subprocess.run(
        [sys.executable, str(generator), "--check"],
        cwd=repo,
        check=False,
    )
    if result.returncode != 0:
        fail("generated FMUv6C profile registry is out of date")

    registry_path = repo / "profiles" / "fmuv6c-profile-registry.json"
    require_file(registry_path)
    try:
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        fail(f"profile registry is invalid JSON: {error}")
    profiles = registry.get("profiles")
    if registry.get("schema_version") != 1 or not isinstance(profiles, list):
        fail("profile registry schema is invalid")
    if registry.get("profile_count") != len(profiles):
        fail("profile registry count differs from its profiles array")
    if len(profiles) != EXPECTED_PROFILE_COUNT:
        fail(
            f"profile registry contains {len(profiles)} profiles, "
            f"expected {EXPECTED_PROFILE_COUNT}"
        )

    seen_ids: set[str] = set()
    seen_fingerprints: set[str] = set()
    validated: list[dict] = []
    for profile in profiles:
        profile_id = profile.get("profile_id")
        fingerprint = profile.get("fingerprint")
        source = profile.get("source")
        if not isinstance(profile_id, str):
            fail("profile registry contains a non-string profile ID")
        try:
            encoded_id = profile_id.encode("ascii")
        except UnicodeEncodeError:
            fail(f"profile ID is not ASCII: {profile_id!r}")
        if not encoded_id or len(encoded_id) >= 48 or profile_id in seen_ids:
            fail(f"profile ID is empty, duplicated, or too long: {profile_id!r}")
        if not isinstance(fingerprint, str) or not re.fullmatch(
            r"[0-9A-F]{16}", fingerprint
        ) or int(fingerprint, 16) == 0 or fingerprint in seen_fingerprints:
            fail(f"profile fingerprint is invalid or duplicated: {fingerprint!r}")
        if not isinstance(source, str) or not source:
            fail(f"profile source is invalid for {profile_id}")
        source_path = repo / source
        require_file(source_path)
        source_bytes = source_path.read_bytes()
        document = json.loads(source_bytes)
        if profile.get("format") != "vehicle_bundle" or document.get("schema_version") != "2.0" or document.get("id") != profile_id:
            fail(f"invalid VehicleBundle schema/identity: {profile_id}")
        if profile.get("bytes") != len(source_bytes):
            fail(f"profile source byte count changed: {profile_id}")
        if profile.get("sha256") != hashlib.sha256(source_bytes).hexdigest():
            fail(f"profile source SHA-256 changed: {profile_id}")
        if fingerprint != f"{fnv1a64(source_bytes):016X}":
            fail(f"profile source FNV fingerprint changed: {profile_id}")
        seen_ids.add(profile_id)
        seen_fingerprints.add(fingerprint)
        validated.append({**profile, "source_path": source_path, "source_bytes": source_bytes})

    if not any(
        profile["profile_id"] == DEFAULT_PROFILE_ID
        for profile in validated
    ):
        fail("safe boot default is absent from the compiled profile registry")
    return registry_path, registry, validated


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository-root", required=True, type=Path)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--firmware-dir", required=True, type=Path)
    parser.add_argument("--toolchain-bin", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    args = parser.parse_args()

    repo = args.repository_root.resolve()
    build = args.build_dir.resolve()
    firmware = args.firmware_dir.resolve()
    registry_path, registry, profiles = load_profile_registry(repo)

    files = {
        extension: firmware / f"{BASENAME}.{extension}"
        for extension in ("elf", "bin", "hex", "map", "px4")
    }
    for path in files.values():
        require_file(path)

    prototype_path = repo / "boards" / "fmu_v6c" / "firmware.prototype"
    prototype = json.loads(prototype_path.read_text(encoding="utf-8"))
    package = json.loads(files["px4"].read_text(encoding="utf-8"))
    for key in ("magic", "board_id", "board_revision", "image_maxsize", "version"):
        if package.get(key) != prototype.get(key):
            fail(f"PX4 package field {key!r} differs from firmware.prototype")
    if package.get("magic") != "PX4FWv1" or package.get("board_id") != BOARD_ID:
        fail("PX4 package magic or FMUv6C board ID is invalid")
    if int(package.get("image_maxsize", 0)) != FLASH_LENGTH:
        fail("package image limit differs from the 1920 KiB application flash region")

    binary = files["bin"].read_bytes()
    if not binary or len(binary) > FLASH_LENGTH:
        fail(f"binary size {len(binary)} is outside the FMUv6C application region")
    padded = binary + b"\xff" * ((-len(binary)) % 4)
    try:
        packaged_image = zlib.decompress(base64.b64decode(package["image"], validate=True))
    except (KeyError, ValueError, zlib.error) as error:
        fail(f"PX4 package image cannot be decoded: {error}")
    if packaged_image != padded:
        fail("PX4 package payload is not the generated BIN image")
    if package.get("image_size") != len(padded):
        fail("PX4 package image_size is incorrect")
    if package.get("image_sha256") != hashlib.sha256(padded).hexdigest():
        fail("PX4 package image_sha256 is incorrect")

    ihex = parse_ihex(files["hex"])
    if min(ihex) != FLASH_ORIGIN:
        fail(f"Intel HEX starts at 0x{min(ihex):08x}, expected 0x{FLASH_ORIGIN:08x}")
    for address, value in ihex.items():
        offset = address - FLASH_ORIGIN
        if offset < 0 or offset >= len(binary) or binary[offset] != value:
            fail(f"Intel HEX differs from BIN at 0x{address:08x}")

    readelf = args.toolchain_bin / "arm-none-eabi-readelf.exe"
    gcc = args.toolchain_bin / "arm-none-eabi-gcc.exe"
    require_file(readelf)
    require_file(gcc)
    elf_header = subprocess.check_output(
        [str(readelf), "-h", str(files["elf"])], text=True, errors="replace"
    )
    entry_match = re.search(r"Entry point address:\s*(0x[0-9a-fA-F]+)", elf_header)
    entry_point = int(entry_match.group(1), 16) if entry_match else 0
    if not (FLASH_ORIGIN <= (entry_point & ~1) < FLASH_ORIGIN + len(binary)):
        fail("ELF entry point is outside the generated FMUv6C flash image")
    if not entry_point & 1:
        fail("ELF entry point does not select the Cortex-M Thumb instruction set")
    if "Machine:" not in elf_header or "ARM" not in elf_header:
        fail("ELF target machine is not ARM")

    elf_sections = subprocess.check_output(
        [str(readelf), "-S", "-W", str(files["elf"])],
        text=True,
        errors="replace",
    )
    text_match = re.search(
        r"\]\s+\.text\s+PROGBITS\s+([0-9a-fA-F]+)\s", elf_sections
    )
    if not text_match or int(text_match.group(1), 16) != FLASH_ORIGIN:
        fail("ELF .text/vector section does not start at 0x08020000")

    initial_stack_pointer, reset_vector = struct.unpack_from("<II", binary)
    ram_ranges = (
        (0x20000000, 0x20020000),
        (0x24000000, 0x24080000),
        (0x30000000, 0x30048000),
        (0x38000000, 0x38010000),
    )
    if initial_stack_pointer & 0x7 or not any(
        start <= initial_stack_pointer <= end for start, end in ram_ranges
    ):
        fail(f"invalid Cortex-M initial stack pointer 0x{initial_stack_pointer:08x}")
    if not reset_vector & 1 or not (
        FLASH_ORIGIN <= (reset_vector & ~1) < FLASH_ORIGIN + len(binary)
    ):
        fail(f"invalid Cortex-M reset vector 0x{reset_vector:08x}")

    map_text = files["map"].read_text(encoding="utf-8", errors="replace")
    flash_pattern = re.compile(
        r"^\s*flash\s+0x0*8020000\s+0x0*1e0000\b", re.MULTILINE | re.IGNORECASE
    )
    if not flash_pattern.search(map_text):
        fail("linker map does not declare the expected 0x08020000/1920 KiB flash region")
    maintenance_symbols = (
        "fmu_v6c_reboot_to_bootloader",
        "HitlSupervisor::handle_bootloader_reboot_command",
        "up_systemreset",
    )
    for symbol in maintenance_symbols:
        if symbol not in map_text:
            fail(f"bootloader maintenance symbol is absent: {symbol}")
    if struct.pack("<I", PX4_BOOT_RTC_SIGNATURE) not in binary:
        fail("PX4 RTC bootloader signature is absent from the application image")

    config_path = build / ".config"
    require_file(config_path)
    config = enabled_config(config_path)
    init_stack_size = config_integer(config_path, "CONFIG_INIT_STACKSIZE")
    hitl_stack_size = config_integer(config_path, "CONFIG_HYDROX_HITL_STACKSIZE")
    if init_stack_size < MIN_HITL_TASK_STACK:
        fail(
            f"init stack {init_stack_size} is below the {MIN_HITL_TASK_STACK}-byte floor"
        )
    if hitl_stack_size < MIN_HITL_TASK_STACK:
        fail(
            f"HITL stack {hitl_stack_size} is below the {MIN_HITL_TASK_STACK}-byte floor"
        )
    (
        max_stack_frame,
        max_stack_function,
        entry_stack_frame,
        dynamic_stack_functions,
        max_dynamic_static_frame,
    ) = verify_stack_usage(build)
    required = {
        "CONFIG_ARM_MPU_EARLY_RESET",
        "CONFIG_DEBUG_FULLOPT",
        "CONFIG_HYDROX_HITL",
        "CONFIG_INTELHEX_BINARY",
        "CONFIG_RAW_BINARY",
        "CONFIG_UART5_RXDMA",
        "CONFIG_UART5_TXDMA",
        "CONFIG_UART7_RXDMA",
        "CONFIG_UART7_TXDMA",
    }
    missing = sorted(required - config)
    if missing:
        fail(f"required HITL configuration is disabled: {', '.join(missing)}")
    if "CONFIG_DEBUG_NOOPT" in config:
        fail("release firmware must not be compiled without optimization")
    physical_output_drivers = {
        "CONFIG_PWM",
        "CONFIG_STM32_PWM",
        "CONFIG_STM32H7_PWM",
        "CONFIG_CAN",
        "CONFIG_NET_CAN",
        "CONFIG_STM32_CAN",
        "CONFIG_STM32_FDCAN",
        "CONFIG_MOTOR",
    }
    enabled_outputs = sorted(config & physical_output_drivers)
    if enabled_outputs:
        fail(f"physical actuator driver is enabled: {', '.join(enabled_outputs)}")
    unexpected_hil_flow_control = sorted(
        config & {"CONFIG_UART7_IFLOWCONTROL", "CONFIG_UART7_OFLOWCONTROL"}
    )
    if unexpected_hil_flow_control:
        fail(
            "TELEM1 must match the host router three-wire contract; "
            f"unexpected flow control: {', '.join(unexpected_hil_flow_control)}"
        )
    for profile in profiles:
        if profile["profile_id"].encode("ascii") not in binary:
            fail(f"compiled profile ID is absent from firmware: {profile['profile_id']}")
        fingerprint_bytes = struct.pack("<Q", int(profile["fingerprint"], 16))
        if fingerprint_bytes not in binary:
            fail(f"compiled profile fingerprint is absent from firmware: {profile['profile_id']}")
        if profile["source_bytes"] not in binary:
            fail(f"compiled profile JSON bytes are absent from firmware: {profile['profile_id']}")

    gcc_version = subprocess.check_output(
        [str(gcc), "--version"], text=True, errors="replace"
    ).splitlines()[0]
    status = git(repo, "status", "--porcelain", "--untracked-files=no")
    outputs = {
        path.name: {"bytes": path.stat().st_size, "sha256": sha256(path)}
        for path in files.values()
    }
    manifest = {
        "schema_version": 1,
        "release_state": "FLASH_CANDIDATE_HITL_ONLY",
        "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "board": {
            "family": "FMUv6C",
            "target": "Pixhawk 6C Mini",
            "board_id": BOARD_ID,
            "board_revision": int(prototype["board_revision"]),
            "flash_origin": f"0x{FLASH_ORIGIN:08X}",
            "image_maxsize": FLASH_LENGTH,
        },
        "firmware": {
            "version": prototype["version"],
            "git_commit": git(repo, "rev-parse", "--verify", "HEAD"),
            "tracked_tree_dirty": bool(status),
            "default_profile": {
                "profile_id": DEFAULT_PROFILE_ID,
                "fingerprint": next(p["fingerprint"] for p in profiles if p["profile_id"] == DEFAULT_PROFILE_ID),
            },
            "profile_registry": {
                "path": str(registry_path.relative_to(repo)).replace("\\", "/"),
                "sha256": sha256(registry_path),
                "profile_count": registry["profile_count"],
                "profiles": [
                    {key: value for key, value in profile.items()
                     if key not in {"source_path", "source_bytes"}}
                    for profile in profiles
                ],
            },
            "binary_bytes": len(binary),
            "binary_utilization_percent": round(100.0 * len(binary) / FLASH_LENGTH, 3),
            "elf_entry_point": f"0x{entry_point:08X}",
            "initial_stack_pointer": f"0x{initial_stack_pointer:08X}",
            "reset_vector": f"0x{reset_vector:08X}",
            "bootloader_maintenance": {
                "handoff_register": "STM32_RTC_BK0R",
                "signature": f"0x{PX4_BOOT_RTC_SIGNATURE:08X}",
                "trigger": "PX4 uploader MAVLink1 broadcast-target pair",
            },
            "optimization": "CONFIG_DEBUG_FULLOPT (-Os)",
            "stack_budget": {
                "init_stack_bytes": init_stack_size,
                "hitl_stack_bytes": hitl_stack_size,
                "max_static_frame_bytes": max_stack_frame,
                "max_static_frame_function": max_stack_function,
                "entry_static_frame_bytes": entry_stack_frame,
                "eigen_dynamic_stack_limit_bytes": EIGEN_DYNAMIC_STACK_LIMIT,
                "eigen_dynamic_stack_functions": dynamic_stack_functions,
                "max_dynamic_stack_static_frame_bytes": max_dynamic_static_frame,
            },
        },
        "dependencies": {
            "nuttx_lock_sha256": sha256(repo / "third_party" / "nuttx.lock.json"),
            "arm_toolchain_lock_sha256": sha256(
                repo / "third_party" / "arm_gnu_toolchain.lock.json"
            ),
            "arm_toolchain": gcc_version,
        },
        "safety": {
            "physical_actuator_drivers_enabled": False,
            "actuator_loads_must_remain_disconnected": True,
            "hardware_qualification_complete": False,
        },
        "outputs": outputs,
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"[OK] FMUv6C HITL flash candidate verified: {len(binary)} / "
        f"{FLASH_LENGTH} bytes ({manifest['firmware']['binary_utilization_percent']}%)"
    )
    print(f"[OK] Release manifest: {args.manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
