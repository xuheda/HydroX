"""Schema-aware XLog 1.0 reader and PINN/RL dataset builder.

The reader validates XLog block CRCs and decodes only the record layouts that
HydroX 1.0 writes. It deliberately uses the file schema and record sizes rather
than assuming a raw contiguous array layout.

Example:
  python learning/training/xlog_dataset.py log/run_001.xlog \
      --output learning/datasets/auv_v1.npz
"""
from __future__ import annotations

import argparse
import glob
import json
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator, Sequence

import numpy as np


FILE_HEADER = struct.Struct("<4sHHHBBIIIIIQQQQQQQQ")
BLOCK_HEADER = struct.Struct("<4sHHIIIIQQQQII")
RECORD_HEADER = struct.Struct("<HHIQQ")

TOPIC_STATE = 1
TOPIC_SETPOINT = 2
TOPIC_CONTROL_ERROR = 3
TOPIC_CONTROLLER_OUTPUT = 4
TOPIC_ACTUATOR = 5
TOPIC_ESTIMATOR_HEALTH = 6
TOPIC_TIMING = 7
TOPIC_SIMULATOR_TRUTH = 8

RECORD_SIZES = {
    TOPIC_STATE: 112,
    TOPIC_SETPOINT: 72,
    TOPIC_CONTROL_ERROR: 40,
    TOPIC_CONTROLLER_OUTPUT: 56,
    TOPIC_ACTUATOR: 72,
    TOPIC_ESTIMATOR_HEALTH: 48,
    TOPIC_TIMING: 32,
    TOPIC_SIMULATOR_TRUTH: 112,
}


@dataclass(frozen=True)
class XLogMetadata:
    path: Path
    metadata: dict[str, Any]
    schema: dict[str, Any]


@dataclass(frozen=True)
class Record:
    topic: int
    timestamp_ns: int
    sequence: int
    value: dict[str, Any]


def _decode_json(raw: bytes, label: str, path: Path) -> dict[str, Any]:
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path}: invalid {label} JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{path}: {label} must be a JSON object")
    return value


def read_metadata(path: str | Path) -> XLogMetadata:
    path = Path(path)
    with path.open("rb") as handle:
        raw = handle.read(FILE_HEADER.size)
        if len(raw) != FILE_HEADER.size:
            raise ValueError(f"{path}: truncated XLog header")
        (magic, major, _minor, header_size, endian, _flags,
         _record_header_size, _block_header_size, _segment_index, _header_crc, _reserved,
         _start_ns, metadata_len, schema_len, _schema_hash, _max_segment, _target_block,
         _file_id_hi, _file_id_lo) = FILE_HEADER.unpack(raw)
        if magic != b"XLOG" or major != 1 or endian != 1:
            raise ValueError(f"{path}: unsupported XLog format")
        if header_size < FILE_HEADER.size:
            raise ValueError(f"{path}: invalid header size {header_size}")
        handle.seek(header_size)
        metadata = _decode_json(handle.read(metadata_len), "metadata", path)
        schema = _decode_json(handle.read(schema_len), "schema", path)
    return XLogMetadata(path=path, metadata=metadata, schema=schema)


def _decode_payload(topic: int, payload: bytes) -> dict[str, Any] | None:
    expected_size = RECORD_SIZES.get(topic)
    if expected_size is None:
        return None
    if len(payload) != expected_size:
        raise ValueError(f"topic {topic} expected {expected_size} bytes, received {len(payload)}")

    if topic == TOPIC_STATE:
        values = struct.unpack("<13d8B", payload)
        return {"eta": np.asarray(values[0:6]), "nu": np.asarray(values[6:12]),
                "depth_m": values[12], "gnc_mode": values[13], "dvl_valid": bool(values[15])}
    if topic == TOPIC_SETPOINT:
        values = struct.unpack("<8d8B", payload)
        return {"depth_ref": values[0], "heading_ref": values[1], "surge_ref": values[2],
                "yaw_rate_ref": values[3], "wp": np.asarray(values[4:7]),
                "setpoint_age_s": values[7], "use_yaw_rate_ref": bool(values[8])}
    if topic == TOPIC_CONTROL_ERROR:
        values = struct.unpack("<5d", payload)
        return {"depth_err": values[0], "heading_err": values[1], "surge_err": values[2],
                "yaw_rate_err": values[3], "wp_dist": values[4]}
    if topic == TOPIC_CONTROLLER_OUTPUT:
        values = struct.unpack("<7d", payload)
        return {"tau": np.asarray(values[0:6]), "tau_norm": values[6]}
    if topic == TOPIC_ACTUATOR:
        values = struct.unpack("<8f4d8B", payload)
        return {"ch": np.asarray(values[0:8], dtype=np.float64), "rpm": values[8],
                "thrust": values[9], "max_abs": values[10], "sat_ratio": values[11],
                "active_count": values[12]}
    if topic == TOPIC_ESTIMATOR_HEALTH:
        values = struct.unpack("<5d8B", payload)
        return {"dvl_age_s": values[0], "accel_norm": values[1], "gyro_norm": values[2],
                "pose_cov_trace": values[3], "twist_cov_trace": values[4],
                "have_accel": bool(values[5]), "dvl_valid": bool(values[6]),
                "gps_valid": bool(values[7])}
    if topic == TOPIC_TIMING:
        values = struct.unpack("<3d8B", payload)
        return {"dt": values[0], "expected_dt": values[1], "setpoint_age_s": values[2],
                "loop_overrun": bool(values[3])}
    if topic == TOPIC_SIMULATOR_TRUTH:
        values = struct.unpack("<12dQ8B", payload)
        return {"eta": np.asarray(values[0:6]), "nu": np.asarray(values[6:12]),
                "source_timestamp_us": values[12], "valid": bool(values[13])}
    return None


def iter_records(path: str | Path, strict_crc: bool = True) -> Iterator[Record]:
    """Yield decoded records in file order, stopping safely at an incomplete tail."""
    path = Path(path)
    with path.open("rb") as handle:
        header_raw = handle.read(FILE_HEADER.size)
        if len(header_raw) != FILE_HEADER.size:
            raise ValueError(f"{path}: truncated XLog header")
        header = FILE_HEADER.unpack(header_raw)
        magic, major, _minor, header_size, endian = header[0:5]
        if magic != b"XLOG" or major != 1 or endian != 1:
            raise ValueError(f"{path}: unsupported XLog format")
        metadata_len, schema_len = header[12], header[13]
        handle.seek(header_size + metadata_len + schema_len)

        while True:
            header_raw = handle.read(BLOCK_HEADER.size)
            if not header_raw:
                return
            if len(header_raw) < 4:
                return
            if header_raw[0:4] == b"XEND":
                return
            if len(header_raw) != BLOCK_HEADER.size:
                return
            (block_magic, block_header_size, _block_version, _flags, payload_size,
             _record_count, payload_crc, _first_ts, _last_ts, _first_seq, _last_seq,
             _header_crc, _reserved) = BLOCK_HEADER.unpack(header_raw)
            if block_magic != b"XBLK" or block_header_size != BLOCK_HEADER.size:
                raise ValueError(f"{path}: malformed XLog block")
            payload = handle.read(payload_size)
            if len(payload) != payload_size:
                return  # Writer intentionally permits an incomplete final block after a crash.
            if strict_crc and (zlib.crc32(payload) & 0xFFFFFFFF) != payload_crc:
                raise ValueError(f"{path}: payload CRC mismatch")

            offset = 0
            while offset < len(payload):
                if len(payload) - offset < RECORD_HEADER.size:
                    raise ValueError(f"{path}: truncated record header")
                topic, _flags, record_size, timestamp_ns, sequence = RECORD_HEADER.unpack_from(payload, offset)
                offset += RECORD_HEADER.size
                if record_size > len(payload) - offset:
                    raise ValueError(f"{path}: truncated record payload")
                raw_record = payload[offset:offset + record_size]
                offset += record_size
                value = _decode_payload(topic, raw_record)
                if value is not None:
                    yield Record(topic, timestamp_ns, sequence, value)


def _wrap_pi(value: float) -> float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def _asof(records: Sequence[Record], timestamps: np.ndarray) -> list[dict[str, Any] | None]:
    """Last-observation-carried-forward alignment for a sparse XLog topic."""
    result: list[dict[str, Any] | None] = []
    index = 0
    latest: dict[str, Any] | None = None
    for timestamp in timestamps:
        while index < len(records) and records[index].timestamp_ns <= timestamp:
            latest = records[index].value
            index += 1
        result.append(latest)
    return result


def _feature_row(state: dict[str, Any], setpoint: dict[str, Any] | None,
                 actuator: dict[str, Any] | None, health: dict[str, Any] | None) -> np.ndarray:
    eta = state["eta"]
    nu = state["nu"]
    sp = setpoint or {}
    depth_ref = float(sp.get("depth_ref", eta[2]))
    surge_ref = float(sp.get("surge_ref", nu[0]))
    yaw_rate_ref = float(sp.get("yaw_rate_ref", nu[5])) if sp.get("use_yaw_rate_ref", False) else nu[5]
    heading_ref = float(sp.get("heading_ref", eta[5]))
    heading_error = _wrap_pi(heading_ref - eta[5])
    act = np.zeros(8, dtype=np.float64) if actuator is None else actuator["ch"]
    dvl_age = 10.0 if health is None else min(max(float(health["dvl_age_s"]), 0.0), 10.0)
    pose_cov = 1.0e6 if health is None else min(max(float(health["pose_cov_trace"]), 0.0), 1.0e6)
    twist_cov = 1.0e6 if health is None else min(max(float(health["twist_cov_trace"]), 0.0), 1.0e6)
    dvl_valid = float(state["dvl_valid"] and health is not None and health["dvl_valid"])
    return np.concatenate((
        nu, np.asarray([eta[2]]),
        np.sin(eta[3:6]), np.cos(eta[3:6]),
        np.asarray([depth_ref - eta[2], surge_ref - nu[0], yaw_rate_ref - nu[5],
                    np.sin(heading_error), np.cos(heading_error)]),
        np.asarray([dvl_valid, dvl_age, pose_cov, twist_cov]), act,
    ))


FEATURE_NAMES = (
    [f"nu_{axis}" for axis in ("u", "v", "w", "p", "q", "r")]
    + ["depth_m"]
    + [f"sin_{axis}" for axis in ("roll", "pitch", "yaw")]
    + [f"cos_{axis}" for axis in ("roll", "pitch", "yaw")]
    + ["depth_error_m", "surge_error_mps", "yaw_rate_error_radps", "sin_heading_error", "cos_heading_error"]
    + ["dvl_valid", "dvl_age_s", "pose_cov_trace", "twist_cov_trace"]
    + [f"actuator_{index}" for index in range(8)]
)


def build_dataset(paths: Sequence[str | Path], output_path: str | Path,
                  min_dt: float = 0.002, max_dt: float = 0.10) -> dict[str, Any]:
    """Build finite, consecutive state transitions from one or more XLog runs."""
    observations: list[np.ndarray] = []
    eta_rows: list[np.ndarray] = []
    depth_rows: list[float] = []
    nu_rows: list[np.ndarray] = []
    next_nu_rows: list[np.ndarray] = []
    tau_rows: list[np.ndarray] = []
    actuator_rows: list[np.ndarray] = []
    setpoint_rows: list[np.ndarray] = []
    mode_rows: list[int] = []
    reset_rows: list[bool] = []
    dt_rows: list[float] = []
    run_rows: list[int] = []
    sources: list[dict[str, Any]] = []

    for run_id, raw_path in enumerate(paths):
        path = Path(raw_path)
        metadata = read_metadata(path)
        records_by_topic: dict[int, list[Record]] = {}
        for record in iter_records(path):
            records_by_topic.setdefault(record.topic, []).append(record)
        states = records_by_topic.get(TOPIC_STATE, [])
        if len(states) < 2:
            raise ValueError(f"{path}: requires at least two HydroxState records")
        timestamps = np.asarray([record.timestamp_ns for record in states], dtype=np.int64)
        aligned = {topic: _asof(records_by_topic.get(topic, []), timestamps)
                   for topic in (TOPIC_SETPOINT, TOPIC_CONTROLLER_OUTPUT, TOPIC_ACTUATOR, TOPIC_ESTIMATOR_HEALTH)}
        accepted = 0
        for index in range(len(states) - 1):
            dt = (timestamps[index + 1] - timestamps[index]) * 1.0e-9
            controller = aligned[TOPIC_CONTROLLER_OUTPUT][index]
            actuator = aligned[TOPIC_ACTUATOR][index]
            setpoint = aligned[TOPIC_SETPOINT][index]
            if (not min_dt <= dt <= max_dt or controller is None or actuator is None or
                    setpoint is None):
                continue
            state = states[index].value
            next_state = states[index + 1].value
            values = (state["eta"], state["nu"], next_state["nu"], controller["tau"], actuator["ch"])
            if not all(np.isfinite(value).all() for value in values):
                continue
            observations.append(_feature_row(state, aligned[TOPIC_SETPOINT][index], actuator,
                                             aligned[TOPIC_ESTIMATOR_HEALTH][index]))
            eta_rows.append(state["eta"])
            depth_rows.append(float(state["depth_m"]))
            nu_rows.append(state["nu"])
            next_nu_rows.append(next_state["nu"])
            tau_rows.append(controller["tau"])
            actuator_rows.append(actuator["ch"])
            setpoint_rows.append(np.asarray((
                setpoint["depth_ref"], setpoint["heading_ref"], setpoint["surge_ref"],
                setpoint["yaw_rate_ref"], setpoint["wp"][0], setpoint["wp"][1],
                setpoint["wp"][2], float(setpoint["use_yaw_rate_ref"]),
            ), dtype=np.float64))
            mode_rows.append(int(state["gnc_mode"]))
            # A source run is an independently started SITL process.  The explicit
            # reset marker lets an offline C++ replay reproduce the controller and
            # residual-filter state boundary without claiming to infer unlogged
            # mid-run setpoint reset events.
            reset_rows.append(accepted == 0)
            dt_rows.append(dt)
            run_rows.append(run_id)
            accepted += 1
        sources.append({"path": str(path), "metadata": metadata.metadata, "samples": accepted})

    if not observations:
        raise ValueError("no valid consecutive state transitions were found")
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_path,
                        observation=np.asarray(observations, dtype=np.float32),
                        eta=np.asarray(eta_rows, dtype=np.float32),
                        depth_m=np.asarray(depth_rows, dtype=np.float32),
                        nu=np.asarray(nu_rows, dtype=np.float32),
                        next_nu=np.asarray(next_nu_rows, dtype=np.float32),
                        tau_base=np.asarray(tau_rows, dtype=np.float32),
                        actuator=np.asarray(actuator_rows, dtype=np.float32),
                        setpoint=np.asarray(setpoint_rows, dtype=np.float32),
                        gnc_mode=np.asarray(mode_rows, dtype=np.int8),
                        reset_controller=np.asarray(reset_rows, dtype=np.bool_),
                        dt=np.asarray(dt_rows, dtype=np.float32),
                        source_run=np.asarray(run_rows, dtype=np.int32),
                        feature_names=np.asarray(FEATURE_NAMES))
    manifest = {"format": "hydrox.pinn-rl-dataset/v2", "samples": len(observations),
                "feature_names": FEATURE_NAMES, "min_dt_s": min_dt, "max_dt_s": max_dt,
                "control_shadow_contract": {
                    "setpoint": ["depth_ref", "heading_ref", "surge_ref", "yaw_rate_ref",
                                 "wp_n", "wp_e", "wp_d", "use_yaw_rate_ref"],
                    "gnc_mode": "HydroX GNCMode integer",
                    "reset_controller": "true only on the first retained row of each source run",
                },
                "sources": sources}
    output_path.with_suffix(output_path.suffix + ".manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def _expand_paths(values: Sequence[str]) -> list[Path]:
    expanded: list[Path] = []
    for value in values:
        matches = [Path(match) for match in glob.glob(value)]
        expanded.extend(matches if matches else [Path(value)])
    return expanded


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a PINN/RL dataset from HydroX XLog 1.0 files")
    parser.add_argument("xlogs", nargs="+", help="XLog paths or glob patterns")
    parser.add_argument("--output", required=True, help="output .npz dataset path")
    parser.add_argument("--min-dt", type=float, default=0.002)
    parser.add_argument("--max-dt", type=float, default=0.10)
    args = parser.parse_args()
    manifest = build_dataset(_expand_paths(args.xlogs), args.output, args.min_dt, args.max_dt)
    print(json.dumps({"samples": manifest["samples"], "output": args.output}, indent=2))


if __name__ == "__main__":
    main()
