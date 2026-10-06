"""
Golden parity vectors for the C++ port.

These are the Python float64 reference behaviour of PidBalance/LqrBalance on
float32-exact inputs with float32-quantized gains. NOT a claim that the
controller balances (PID sign/stability untested, #359; PID and LQR give
opposite output signs for the same tilt, as in the sources). NOT MCU or
hardware evidence. The C++ copies are written/checked by export_cpp.py (#353),
not here. reset=True means call reset() on the PID BEFORE applying that step.
"""

import argparse
import itertools
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hal.hal import HAL_CONTRACT_VERSION
from hal.gen_lqr_header import SCHEMA, f32, fmt_f32, gains_hash, format_hash, default_gains


HERE = Path(__file__).resolve().parent
DEFAULT_JSON = HERE / "parity_vectors.json"
REAL_DT = (0.016, 0.016, 0.016, 0.032)
STRESS_DT = (0.016, 0.032)
NOMINAL_DT = 0.02

LQR_INPUTS = (
    (-0.1086, 0.0, 0.02),    # 0
    (0.0, 0.0, 0.02),        # 1
    (-0.1076, 0.0, 0.02),    # 2
    (-0.1096, 0.0, 0.02),    # 3
    (-0.1086, 0.01, 0.02),   # 4
    (-0.1086, -0.01, 0.02),  # 5
    (-0.1086, 0.1, 0.02),    # 6
    (-0.1086, -0.1, 0.02),   # 7
    (-0.0986, 0.1, 0.02),    # 8
    (-0.1186, -0.1, 0.02),   # 9
    (-0.0586, 0.0, 0.016),   # 10
    (-0.1586, 0.0, 0.016),   # 11
    (-0.0670, 0.0, 0.016),   # 12
    (-0.0660, 0.0, 0.016),   # 13
    (-0.1502, 0.0, 0.032),   # 14
    (-0.1512, 0.0, 0.032),   # 15
    (0.0914, 0.5, 0.02),     # 16
    (-0.2086, -1.5, 0.02),   # 17
    (-0.0986, 0.1, 0.0),     # 18
    (-0.0986, 0.1, 0.016),   # 19
    (-0.0986, 0.1, 0.032),   # 20
    (-0.1086, 0.5, 0.02),    # 21
    (-0.0986, 0.1, -0.001),  # 22
    (-0.0586, 0.1, -1.0),    # 23
)


def _pid_inputs():
    """
    Generate deterministic PID input sequence: list of (pitch, dt, reset) tuples.

    74 steps total: Segment 1 (45 steps) with P1-P8, Segment 2 (29 steps) with Q1-Q4.
    No RNG, no math functions. real/stress iterators cycle through REAL_DT/STRESS_DT.
    """
    result = []
    real = itertools.cycle(REAL_DT)
    stress = itertools.cycle(STRESS_DT)

    # Segment 1: 45 steps
    # P1: 10 steps
    p1_pitches = [0.0, -0.01, -0.02, -0.015, 0.0, 0.01, 0.02, 0.015, 0.005, -0.005]
    result.append((p1_pitches[0], NOMINAL_DT, True))  # step 0: reset=True
    for i in range(1, 10):
        result.append((p1_pitches[i], next(real), False))

    # P2: 2 steps, dt=0.0 (does NOT consume real)
    result.append((-0.05, 0.0, False))
    result.append((-0.08, 0.0, False))

    # P3: 2 steps
    result.append((-0.03, next(real), False))
    result.append((-0.02, next(real), False))

    # P4: 1 step, dt=-0.01 (fault, does NOT consume real)
    result.append((-0.02, -0.01, False))

    # P5: 2 steps
    result.append((-0.02, next(real), False))
    result.append((-0.01, next(real), False))

    # P6: 3 steps
    result.append((-1.0, next(real), False))
    result.append((-2.0, next(real), False))
    result.append((-3.0, next(real), False))

    # P7: 22 steps, pitch=-3.0
    for _ in range(22):
        result.append((-3.0, next(real), False))

    # P8: 3 steps
    result.append((-2.0, next(real), False))
    result.append((-1.0, next(real), False))
    result.append((0.0, next(real), False))

    # Segment 2: 29 steps
    # Q1: 1 step, reset=True (mid-sequence reset)
    result.append((0.0, NOMINAL_DT, True))

    # Q2: 3 steps
    result.append((1.0, next(stress), False))
    result.append((2.0, next(stress), False))
    result.append((3.0, next(stress), False))

    # Q3: 22 steps, pitch=3.0
    for _ in range(22):
        result.append((3.0, next(stress), False))

    # Q4: 3 steps
    result.append((2.0, next(stress), False))
    result.append((1.0, next(stress), False))
    result.append((0.0, next(stress), False))

    return result


def build_vectors():
    """
    Build golden parity vectors dict.

    Lazily imports control_core, hal.hal.ImuSample, f32_emulation.
    Returns dict with FIXED key order: schema, contract_version, gains_hash, tol_abs,
    note, lqr (with K, theta_ref_rad, out_limit, cases), pid (with kp, ki, kd,
    out_limit, sequence).
    """
    from hal.hal import ImuSample
    from hal.control_core import PidBalance, LqrBalance
    from hal.f32_emulation import F32_TOL_ABS

    g = default_gains()

    # Quantize every input with f32()
    lqr_cases = []
    core = LqrBalance(K=g["K"], theta_ref_rad=g["theta_ref"], out_limit=g["out_limit"])
    for pitch, gyro_y, dt in LQR_INPUTS:
        pitch_q = f32(pitch)
        gyro_y_q = f32(gyro_y)
        dt_q = f32(dt)
        out = core.step(ImuSample(0.0, 0.0, pitch_q, 0.0, (0.0, gyro_y_q, 0.0)), dt_q)
        lqr_cases.append({
            "pitch": pitch_q,
            "gyro_y": gyro_y_q,
            "dt": dt_q,
            "cmd": float(out.cmd_rad_s),
            "raw": float(out.raw_cmd),
            "fault": out.fault,
        })

    # PID: build sequence with debug tracking
    kp, ki, kd, limit = g["pid"]
    pid_core = PidBalance(kp=kp, ki=ki, kd=kd, out_limit=limit)
    pid_sequence = []
    pid_debug = []

    for pitch, dt, reset in _pid_inputs():
        pitch_q = f32(pitch)
        dt_q = f32(dt)
        if reset:
            pid_core.reset()
        out = pid_core.step(ImuSample(0.0, 0.0, pitch_q, 0.0, (0.0, 0.0, 0.0)), dt_q)
        pid_sequence.append({
            "pitch": pitch_q,
            "dt": dt_q,
            "cmd": float(out.cmd_rad_s),
            "raw": float(out.raw_cmd),
            "reset": reset,
            "fault": out.fault,
        })
        pid_debug.append(out.debug)

    # Build final dict with FIXED key order
    result = {
        "schema": SCHEMA,
        "contract_version": g["contract_version"],
        "gains_hash": format_hash(gains_hash(**g)),
        "tol_abs": F32_TOL_ABS,
        "note": "Reference behaviour of the Python float64 core on float32-exact inputs; not a claim the controller balances (PID sign/stability untested, #359); not MCU/hardware evidence.",
        "lqr": {
            "K": list(g["K"]),
            "theta_ref_rad": g["theta_ref"],
            "out_limit": g["out_limit"],
            "cases": lqr_cases,
        },
        "pid": {
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "out_limit": limit,
            "sequence": pid_sequence,
        },
    }

    _assert_coverage(result, pid_debug)
    return result


def _assert_coverage(v, pid_debug):
    """
    Validate parity vectors coverage. Raise AssertionError on failure.

    LQR: at least one zero case, saturation both signs, near-boundary pair,
    at least 2 faults, at least one dt==0.
    PID: n>=50, saturation both signs, integral clamped both signs,
    at least one dt==0, at least one dt<0 (fault), reset at index 0 and >=1 more,
    at least one fault.
    """
    # LQR coverage
    lqr_cases = v["lqr"]["cases"]

    # Check: at least one with cmd==0.0 and raw==0.0 and not fault
    has_zero_case = any(
        c["cmd"] == 0.0 and c["raw"] == 0.0 and not c["fault"]
        for c in lqr_cases
    )
    assert has_zero_case, "LQR: no zero case found"

    # Check: saturated both signs
    out_limit = v["lqr"]["out_limit"]
    has_pos_sat = any(
        c["cmd"] == out_limit and c["raw"] > out_limit and not c["fault"]
        for c in lqr_cases
    )
    has_neg_sat = any(
        c["cmd"] == -out_limit and c["raw"] < -out_limit and not c["fault"]
        for c in lqr_cases
    )
    assert has_pos_sat, "LQR: no positive saturation case"
    assert has_neg_sat, "LQR: no negative saturation case"

    # Check: near-boundary pair (0.9 < abs(raw) < 1.0 and 1.0 < abs(raw) < 1.1)
    near_low = any(0.9 < abs(c["raw"]) < 1.0 for c in lqr_cases)
    near_high = any(1.0 < abs(c["raw"]) < 1.1 for c in lqr_cases)
    assert near_low and near_high, "LQR: no near-boundary pair"

    # Check: at least 2 fault cases
    fault_count = sum(1 for c in lqr_cases if c["fault"])
    assert fault_count >= 2, f"LQR: only {fault_count} fault case(s), need >=2"

    # Check: at least one dt==0
    has_dt_zero = any(c["dt"] == 0.0 for c in lqr_cases)
    assert has_dt_zero, "LQR: no dt==0 case"

    # PID coverage
    pid_seq = v["pid"]["sequence"]
    ki = v["pid"]["ki"]
    pid_limit = v["pid"]["out_limit"]

    # Check: n >= 50
    n = len(pid_seq)
    assert n >= 50, f"PID: sequence length {n} < 50"

    # Check: saturation both signs
    has_pid_pos_sat = any(
        s["raw"] > pid_limit and s["cmd"] == pid_limit and not s["fault"]
        for s in pid_seq
    )
    has_pid_neg_sat = any(
        s["raw"] < -pid_limit and s["cmd"] == -pid_limit and not s["fault"]
        for s in pid_seq
    )
    assert has_pid_pos_sat, "PID: no positive saturation"
    assert has_pid_neg_sat, "PID: no negative saturation"

    # Check: integral clamped both signs
    # abs(debug[2] - ki*limit) < 1e-12 and debug[2] > 0 (positive clamp)
    # abs(abs(debug[2]) - ki*limit) < 1e-12 and debug[2] < 0 (negative clamp)
    has_pid_int_pos = any(
        abs(debug[2] - ki * pid_limit) < 1e-12 and debug[2] > 0
        for debug in pid_debug
    )
    has_pid_int_neg = any(
        abs(abs(debug[2]) - ki * pid_limit) < 1e-12 and debug[2] < 0
        for debug in pid_debug
    )
    assert has_pid_int_pos, "PID: integral not clamped positive"
    assert has_pid_int_neg, "PID: integral not clamped negative"

    # Check: at least one dt==0
    has_pid_dt_zero = any(s["dt"] == 0.0 for s in pid_seq)
    assert has_pid_dt_zero, "PID: no dt==0 case"

    # Check: at least one dt<0 (fault)
    has_pid_dt_neg = any(s["dt"] < 0.0 for s in pid_seq)
    assert has_pid_dt_neg, "PID: no dt<0 case"

    # Check: reset at index 0 and at least one more
    reset_count = sum(1 for s in pid_seq if s["reset"])
    assert pid_seq[0]["reset"], "PID: sequence[0] reset not True"
    assert reset_count >= 2, f"PID: only {reset_count} reset(s), need >=2"

    # Check: at least one fault
    has_pid_fault = any(s["fault"] for s in pid_seq)
    assert has_pid_fault, "PID: no fault case"


def render_json(v):
    """Convert vectors dict to JSON string with indent=2 and trailing newline."""
    return json.dumps(v, indent=2, allow_nan=False) + "\n"


def load_vectors(path=DEFAULT_JSON):
    """Load vectors dict from JSON file."""
    return json.loads(Path(path).read_text(encoding="utf-8"))


def gains_args(v):
    """Extract gains dict from vectors dict."""
    return {
        "K": tuple(v["lqr"]["K"]),
        "theta_ref": v["lqr"]["theta_ref_rad"],
        "out_limit": v["lqr"]["out_limit"],
        "pid": (v["pid"]["kp"], v["pid"]["ki"], v["pid"]["kd"], v["pid"]["out_limit"]),
        "contract_version": v["contract_version"],
    }


def validate_vectors(v):
    """
    Validate vectors dict. STDLIB ONLY (no numpy).

    Returns list of problem strings; empty means OK.
    """
    import math
    import re

    problems = []

    # Check schema
    if v.get("schema") != SCHEMA:
        problems.append(f"schema mismatch: got {v.get('schema')}, expected {SCHEMA}")

    # Check contract_version
    if v.get("contract_version") != HAL_CONTRACT_VERSION:
        problems.append(
            f"contract_version mismatch: got {v.get('contract_version')}, "
            f"expected {HAL_CONTRACT_VERSION}"
        )

    # Check K length
    K = v.get("lqr", {}).get("K", [])
    if len(K) != 2:
        problems.append(f"K length {len(K)} != 2")

    # Check gains_hash format and value
    g_hash = v.get("gains_hash", "")
    if not re.match(r"^0x[0-9a-f]{8}$", g_hash):
        problems.append(f"gains_hash format invalid: {g_hash}")
    else:
        try:
            computed_hash = format_hash(gains_hash(**gains_args(v)))
            if computed_hash != g_hash:
                problems.append(f"gains_hash mismatch: got {g_hash}, expected {computed_hash}")
        except Exception as e:
            problems.append(f"gains_hash computation failed: {e}")

    # Check all gain/input values satisfy f32(x)==x
    def check_f32(name, x):
        if not math.isfinite(x):
            problems.append(f"{name} not finite: {x}")
        elif f32(x) != x:
            problems.append(f"{name} not f32-exact: {x}")

    try:
        check_f32("theta_ref_rad", v["lqr"]["theta_ref_rad"])
        check_f32("out_limit", v["lqr"]["out_limit"])
        for i, k in enumerate(K):
            check_f32(f"K[{i}]", k)

        check_f32("pid.kp", v["pid"]["kp"])
        check_f32("pid.ki", v["pid"]["ki"])
        check_f32("pid.kd", v["pid"]["kd"])
        check_f32("pid.out_limit", v["pid"]["out_limit"])

        for i, case in enumerate(v["lqr"]["cases"]):
            check_f32(f"lqr.cases[{i}].pitch", case["pitch"])
            check_f32(f"lqr.cases[{i}].gyro_y", case["gyro_y"])
            check_f32(f"lqr.cases[{i}].dt", case["dt"])

        for i, step in enumerate(v["pid"]["sequence"]):
            check_f32(f"pid.sequence[{i}].pitch", step["pitch"])
            check_f32(f"pid.sequence[{i}].dt", step["dt"])
    except Exception as e:
        problems.append(f"f32-exact check failed: {e}")

    # Check all cmd/raw are finite
    try:
        for i, case in enumerate(v["lqr"]["cases"]):
            if not math.isfinite(case["cmd"]):
                problems.append(f"lqr.cases[{i}].cmd not finite: {case['cmd']}")
            if not math.isfinite(case["raw"]):
                problems.append(f"lqr.cases[{i}].raw not finite: {case['raw']}")

        for i, step in enumerate(v["pid"]["sequence"]):
            if not math.isfinite(step["cmd"]):
                problems.append(f"pid.sequence[{i}].cmd not finite: {step['cmd']}")
            if not math.isfinite(step["raw"]):
                problems.append(f"pid.sequence[{i}].raw not finite: {step['raw']}")
    except Exception as e:
        problems.append(f"finite check failed: {e}")

    # Check non-fault rows: abs(cmd) <= out_limit (+1e-12) and cmd clamping
    out_limit = v["lqr"]["out_limit"]
    try:
        for i, case in enumerate(v["lqr"]["cases"]):
            if not case["fault"]:
                if abs(case["cmd"]) > out_limit + 1e-12:
                    problems.append(
                        f"lqr.cases[{i}]: cmd {case['cmd']} exceeds limit {out_limit}"
                    )
                expected_cmd = max(-out_limit, min(out_limit, case["raw"]))
                if abs(case["cmd"] - expected_cmd) > 1e-12:
                    problems.append(
                        f"lqr.cases[{i}]: cmd {case['cmd']} != clamp({case['raw']}) = {expected_cmd}"
                    )
    except Exception as e:
        problems.append(f"LQR clamping check failed: {e}")

    pid_limit = v["pid"]["out_limit"]
    try:
        for i, step in enumerate(v["pid"]["sequence"]):
            if not step["fault"]:
                if abs(step["cmd"]) > pid_limit + 1e-12:
                    problems.append(
                        f"pid.sequence[{i}]: cmd {step['cmd']} exceeds limit {pid_limit}"
                    )
                expected_cmd = max(-pid_limit, min(pid_limit, step["raw"]))
                if abs(step["cmd"] - expected_cmd) > 1e-12:
                    problems.append(
                        f"pid.sequence[{i}]: cmd {step['cmd']} != clamp({step['raw']}) = {expected_cmd}"
                    )
    except Exception as e:
        problems.append(f"PID clamping check failed: {e}")

    # Check dt<0 implies fault, fault implies cmd==0 and raw==0
    try:
        for i, case in enumerate(v["lqr"]["cases"]):
            if case["dt"] < 0.0 and not case["fault"]:
                problems.append(f"lqr.cases[{i}]: dt<0 but fault=False")
            if case["fault"]:
                if case["cmd"] != 0.0 or case["raw"] != 0.0:
                    problems.append(
                        f"lqr.cases[{i}]: fault=True but cmd={case['cmd']}, raw={case['raw']}"
                    )

        for i, step in enumerate(v["pid"]["sequence"]):
            if step["dt"] < 0.0 and not step["fault"]:
                problems.append(f"pid.sequence[{i}]: dt<0 but fault=False")
            if step["fault"]:
                if step["cmd"] != 0.0 or step["raw"] != 0.0:
                    problems.append(
                        f"pid.sequence[{i}]: fault=True but cmd={step['cmd']}, raw={step['raw']}"
                    )
    except Exception as e:
        problems.append(f"fault consistency check failed: {e}")

    # Check lengths
    if len(v["lqr"]["cases"]) < 20:
        problems.append(f"lqr.cases length {len(v['lqr']['cases'])} < 20")
    if len(v["pid"]["sequence"]) < 50:
        problems.append(f"pid.sequence length {len(v['pid']['sequence'])} < 50")

    # Check: a fault flag only appears with dt<0 (dt<0 iff fault)
    try:
        for i, case in enumerate(v["lqr"]["cases"]):
            if case["fault"] and case["dt"] >= 0.0:
                problems.append(f"lqr.cases[{i}]: fault=True but dt>=0")
        for i, step in enumerate(v["pid"]["sequence"]):
            if step["fault"] and step["dt"] >= 0.0:
                problems.append(f"pid.sequence[{i}]: fault=True but dt>=0")
    except Exception as e:
        problems.append(f"fault/dt check failed: {e}")

    # Check: the PID sequence starts with a reset (C++ must reset BEFORE step 0)
    try:
        if v["pid"]["sequence"][0]["reset"] is not True:
            problems.append("pid.sequence[0].reset is not True")
    except Exception as e:
        problems.append(f"pid.sequence[0].reset check failed: {e}")

    # Check tol_abs > 0
    if v.get("tol_abs", 0) <= 0:
        problems.append(f"tol_abs {v.get('tol_abs')} <= 0")

    return problems


# --- PART 2 (f32_agreement, render_unity_inc, count_inc_rows, check, main) appended below ---


def f32_agreement(v) -> dict:
    """
    Test float32 agreement: run float32 emulation on parity vectors.

    Lazily imports Float32LqrBalance, Float32PidBalance, ImuSample.
    Returns dict with lqr_max_abs_cmd, lqr_max_abs_raw, pid_max_abs_cmd, pid_max_abs_raw.
    """
    from hal.hal import ImuSample
    from hal.f32_emulation import Float32LqrBalance, Float32PidBalance

    # LQR agreement
    lqr_max_abs_cmd = 0.0
    lqr_max_abs_raw = 0.0
    lqr_core = Float32LqrBalance(
        K=tuple(v["lqr"]["K"]),
        theta_ref_rad=v["lqr"]["theta_ref_rad"],
        out_limit=v["lqr"]["out_limit"]
    )
    for case in v["lqr"]["cases"]:
        out = lqr_core.step(
            ImuSample(0.0, 0.0, case["pitch"], 0.0, (0.0, case["gyro_y"], 0.0)),
            case["dt"]
        )
        lqr_max_abs_cmd = max(lqr_max_abs_cmd, abs(out.cmd_rad_s - case["cmd"]))
        lqr_max_abs_raw = max(lqr_max_abs_raw, abs(out.raw_cmd - case["raw"]))

    # PID agreement
    pid_max_abs_cmd = 0.0
    pid_max_abs_raw = 0.0
    pid_core = Float32PidBalance(
        kp=v["pid"]["kp"],
        ki=v["pid"]["ki"],
        kd=v["pid"]["kd"],
        out_limit=v["pid"]["out_limit"]
    )
    for step in v["pid"]["sequence"]:
        if step["reset"]:
            pid_core.reset()
        out = pid_core.step(
            ImuSample(0.0, 0.0, step["pitch"], 0.0, (0.0, 0.0, 0.0)),
            step["dt"]
        )
        pid_max_abs_cmd = max(pid_max_abs_cmd, abs(out.cmd_rad_s - step["cmd"]))
        pid_max_abs_raw = max(pid_max_abs_raw, abs(out.raw_cmd - step["raw"]))

    return {
        "lqr_max_abs_cmd": lqr_max_abs_cmd,
        "lqr_max_abs_raw": lqr_max_abs_raw,
        "pid_max_abs_cmd": pid_max_abs_cmd,
        "pid_max_abs_raw": pid_max_abs_raw,
    }


def render_unity_inc(v) -> str:
    """
    Render C++ include header for Unity/parity verification.

    Pure function (stdlib + fmt_f32). Returns lines joined with newline plus trailing newline.
    No double-quote characters; all booleans lowercase true/false.
    Negative-zero normalized to positive-zero before fmt_f32.
    """
    lines = [
        "// GENERATED by freecad-workspace gen_parity_vectors.py via export_cpp.py -- do not edit.",
        "// Reference behaviour of the Python float64 core on float32-exact inputs. NOT a claim that the",
        "// controller balances (PID sign/stability untested, freecad-workspace#359). NOT MCU or hardware evidence.",
        "// PidStep.reset: call reset() on the PID BEFORE applying this step. Gains are SIM-derived.",
        "#include <cstdint>",
        f"constexpr uint32_t kVecSchema = {v['schema']}, kVecContractVersion = {v['contract_version']}, kVecGainsHash = {v['gains_hash']}u;",
        f"constexpr float kTolAbs = {fmt_f32(v['tol_abs'])};",
        "struct LqrCase { float pitch, gyro_y, dt, cmd, raw; bool fault; };",
        "struct PidStep { float pitch, dt, cmd; bool reset; float raw; bool fault; };",
        "constexpr LqrCase kLqrCases[] = {",
    ]

    for case in v["lqr"]["cases"]:
        pitch = 0.0 if case["pitch"] == 0.0 else case["pitch"]
        gyro_y = 0.0 if case["gyro_y"] == 0.0 else case["gyro_y"]
        dt = 0.0 if case["dt"] == 0.0 else case["dt"]
        cmd = 0.0 if case["cmd"] == 0.0 else case["cmd"]
        raw = 0.0 if case["raw"] == 0.0 else case["raw"]
        fault_str = "true" if case["fault"] else "false"
        lines.append(
            f"  {{ {fmt_f32(pitch)}, {fmt_f32(gyro_y)}, {fmt_f32(dt)}, "
            f"{fmt_f32(cmd)}, {fmt_f32(raw)}, {fault_str} }},"
        )

    lines.append("};")
    lines.append("constexpr PidStep kPidSeq[] = {")

    for step in v["pid"]["sequence"]:
        pitch = 0.0 if step["pitch"] == 0.0 else step["pitch"]
        dt = 0.0 if step["dt"] == 0.0 else step["dt"]
        cmd = 0.0 if step["cmd"] == 0.0 else step["cmd"]
        raw = 0.0 if step["raw"] == 0.0 else step["raw"]
        reset_str = "true" if step["reset"] else "false"
        fault_str = "true" if step["fault"] else "false"
        lines.append(
            f"  {{ {fmt_f32(pitch)}, {fmt_f32(dt)}, {fmt_f32(cmd)}, "
            f"{reset_str}, {fmt_f32(raw)}, {fault_str} }},"
        )

    lines.append("};")

    return "\n".join(lines) + "\n"


def count_inc_rows(inc) -> tuple:
    """
    Count LQR and PID rows in generated C++ include.

    Splits at 'kLqrCases[]' and 'kPidSeq[]', counts lines starting with '  {'
    in each segment. Returns (n_lqr, n_pid).
    """
    parts = inc.split("kPidSeq[]")
    if len(parts) != 2:
        return (0, 0)

    lqr_segment = parts[0]
    pid_segment = parts[1]

    lqr_lines = [line for line in lqr_segment.split("\n") if line.lstrip().startswith("{")]
    pid_lines = [line for line in pid_segment.split("\n") if line.lstrip().startswith("{")]

    return (len(lqr_lines), len(pid_lines))


def check(json_path=DEFAULT_JSON) -> int:
    """
    Validate parity vectors against regeneration, float32 agreement, and tolerance.

    Returns 0 (OK), 1 (validation error), or 2 (file not found).
    """
    if not Path(json_path).exists():
        print(f"vectors file not found: {json_path}", file=sys.stderr)
        return 2

    committed_text = Path(json_path).read_text(encoding="utf-8")
    v = json.loads(committed_text)

    problems = validate_vectors(v)

    fresh = render_json(build_vectors())
    if fresh != committed_text:
        problems.append("committed vectors differ from regeneration (drift)")

    from hal.f32_emulation import F32_TOL_ABS

    if v["tol_abs"] != F32_TOL_ABS:
        problems.append(
            f"tol_abs mismatch: got {v['tol_abs']}, expected {F32_TOL_ABS}"
        )

    agreement = f32_agreement(v)

    if agreement["lqr_max_abs_cmd"] > v["tol_abs"]:
        problems.append(
            f"lqr_max_abs_cmd {agreement['lqr_max_abs_cmd']} > tol_abs {v['tol_abs']}"
        )
    if agreement["lqr_max_abs_raw"] > v["tol_abs"]:
        problems.append(
            f"lqr_max_abs_raw {agreement['lqr_max_abs_raw']} > tol_abs {v['tol_abs']}"
        )
    if agreement["pid_max_abs_cmd"] > v["tol_abs"]:
        problems.append(
            f"pid_max_abs_cmd {agreement['pid_max_abs_cmd']} > tol_abs {v['tol_abs']}"
        )
    if agreement["pid_max_abs_raw"] > v["tol_abs"]:
        problems.append(
            f"pid_max_abs_raw {agreement['pid_max_abs_raw']} > tol_abs {v['tol_abs']}"
        )

    inc_text = render_unity_inc(v)
    counts = count_inc_rows(inc_text)
    if counts != (len(v["lqr"]["cases"]), len(v["pid"]["sequence"])):
        problems.append(
            f"include row counts {counts} != vector lengths "
            f"({len(v['lqr']['cases'])}, {len(v['pid']['sequence'])})"
        )

    if problems:
        for problem in problems:
            print(problem, file=sys.stderr)
        return 1

    print(
        f"OK: parity vectors valid, match regeneration, float32 agreement <= tol_abs "
        f"(Python-only; C++ copies NOT checked); "
        f"lqr_max_abs_cmd={agreement['lqr_max_abs_cmd']}, "
        f"lqr_max_abs_raw={agreement['lqr_max_abs_raw']}, "
        f"pid_max_abs_cmd={agreement['pid_max_abs_cmd']}, "
        f"pid_max_abs_raw={agreement['pid_max_abs_raw']}"
    )
    return 0


def main(argv=None) -> int:
    """
    Argparse-based CLI for parity vector generation, validation, and C++ emission.

    Modes:
      No flags: regenerate JSON and write to --json PATH
      --check: validate JSON against regeneration and float32 agreement
      --emit-inc PATH: write C++ include header to PATH
    """
    parser = argparse.ArgumentParser(
        description="Generate and validate parity vectors"
    )
    parser.add_argument(
        "--json",
        type=str,
        default=str(DEFAULT_JSON),
        help=f"Path to parity vectors JSON (default: {DEFAULT_JSON})",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate JSON against regeneration and float32 agreement",
    )
    parser.add_argument(
        "--emit-inc",
        type=str,
        help="Write C++ include header to this path",
    )

    args = parser.parse_args(argv)

    if args.check:
        return check(args.json)

    if args.emit_inc:
        try:
            v = load_vectors(args.json)
            problems = validate_vectors(v)
            if problems:
                for problem in problems:
                    print(problem, file=sys.stderr)
                return 1
            inc_text = render_unity_inc(v)
            with open(args.emit_inc, "w", newline="\n", encoding="ascii") as f:
                f.write(inc_text)
            print(args.emit_inc)
            return 0
        except Exception as e:
            print(f"Error emitting include: {e}", file=sys.stderr)
            return 1

    # Default: regenerate JSON
    try:
        fresh = render_json(build_vectors())
        json_path = Path(args.json)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        with open(json_path, "w", newline="\n", encoding="utf-8") as f:
            f.write(fresh)
        v = json.loads(fresh)
        print(json_path)
        print(
            f"lqr_cases={len(v['lqr']['cases'])}, "
            f"pid_sequence={len(v['pid']['sequence'])}"
        )
        return 0
    except Exception as e:
        print(f"Error generating vectors: {e}", file=sys.stderr)
        return 1


__all__ = [
    "DEFAULT_JSON",
    "REAL_DT",
    "STRESS_DT",
    "NOMINAL_DT",
    "LQR_INPUTS",
    "build_vectors",
    "render_json",
    "load_vectors",
    "gains_args",
    "validate_vectors",
    "f32_agreement",
    "render_unity_inc",
    "count_inc_rows",
    "check",
    "main",
]


if __name__ == "__main__":
    sys.exit(main())
