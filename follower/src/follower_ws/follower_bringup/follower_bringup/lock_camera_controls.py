#!/usr/bin/env python3
"""Force the V4L2 control lock after the camera node has opened the device.

v4l2_camera exposes the controls as ROS parameters, but the order in which it
applies them is not guaranteed, and several of this camera's controls are
INACTIVE until their "automatic" partner is switched off:

    focus_absolute          is inactive while focus_automatic_continuous = 1
    exposure_time_absolute  is inactive while auto_exposure             = 3
    white_balance_temperature is inactive while white_balance_automatic = 1

Applying the automatic flags first and the manual values second is the only
ordering that works, so it is done explicitly here and then READ BACK. A
control that silently refused to change is worse than one that never tried,
because the calibration downstream would be quietly invalid.

Run standalone or from the camera launch file a couple of seconds after the
camera node starts.
"""

from __future__ import annotations

import argparse
import subprocess
import sys

# Applied in this exact order. The value here is only a fallback: whatever
# camera.yaml says wins, because the camera node is handed the same file and
# two disagreeing copies of a control value is precisely the failure this
# script exists to catch. (An earlier version read only focus and exposure
# from the YAML and hardcoded the rest, so raising `gain` in camera.yaml was
# silently undone here and then reported as "ok".)
LOCK_SEQUENCE: list[tuple[str, int]] = [
    ("focus_automatic_continuous", 0),
    ("auto_exposure", 1),               # 1 = Manual Mode
    ("white_balance_automatic", 0),
    ("exposure_dynamic_framerate", 0),
    ("focus_absolute", 0),
    ("exposure_time_absolute", 250),
    ("white_balance_temperature", 4000),
    ("backlight_compensation", 0),
    ("gain", 0),
    ("zoom_absolute", 100),
    ("pan_absolute", 0),
    ("tilt_absolute", 0),
]

# Controls whose value must come from the YAML when the YAML names them.
YAML_DRIVEN = {name for name, _ in LOCK_SEQUENCE}


def resolve_sequence(params_file: str | None,
                     focus: int | None = None,
                     exposure: int | None = None) -> list[tuple[str, int]]:
    """Decide the value for every locked control.

    camera.yaml is the single source of truth: any control it names wins over
    the fallback in LOCK_SEQUENCE. Command-line --focus/--exposure win over
    both, for one-off experiments.
    """
    from_yaml: dict[str, int] = {}
    if params_file:
        try:
            import yaml
            with open(params_file, "r", encoding="utf-8") as stream:
                document = yaml.safe_load(stream) or {}
            for node in document.values():
                params = (node or {}).get("ros__parameters", {})
                for key in YAML_DRIVEN:
                    if key not in params:
                        continue
                    value = params[key]
                    # v4l2_camera types bool controls as true/false; the
                    # device wants 1/0.
                    from_yaml[key] = (1 if value else 0) if isinstance(value, bool) \
                        else int(value)
        except (OSError, ValueError, TypeError, ImportError) as exc:
            print(f"could not read {params_file}: {exc}", file=sys.stderr)

    explicit = {"focus_absolute": focus, "exposure_time_absolute": exposure}

    def wanted(name: str, fallback: int) -> int:
        if explicit.get(name) is not None:
            return int(explicit[name])
        if name in from_yaml:
            return from_yaml[name]
        return fallback

    return [(name, wanted(name, value)) for name, value in LOCK_SEQUENCE]


def current_controls(device: str) -> dict[str, str]:
    result = subprocess.run(["v4l2-ctl", "-d", device, "-l"],
                            capture_output=True, text=True, check=False)
    values: dict[str, str] = {}
    for line in result.stdout.splitlines():
        if "value=" not in line:
            continue
        name = line.strip().split()[0]
        for token in line.split():
            if token.startswith("value="):
                values[name] = token.split("=", 1)[1]
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--params-file", default=None,
                        help="camera.yaml to read focus/exposure from, so the YAML "
                             "stays the single source of truth")
    parser.add_argument("--focus", type=int, default=None,
                        help="Override focus_absolute (see 07_check_camera.py --focus-sweep)")
    parser.add_argument("--exposure", type=int, default=None,
                        help="Override exposure_time_absolute")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    sequence = resolve_sequence(args.params_file, args.focus, args.exposure)

    for name, value in sequence:
        subprocess.run(["v4l2-ctl", "-d", args.device, f"--set-ctrl={name}={value}"],
                       capture_output=True, check=False)

    after = current_controls(args.device)
    failures: list[str] = []
    for name, value in sequence:
        got = after.get(name)
        if got is None:
            continue                      # control not present on this camera
        if got != str(value):
            failures.append(f"{name}: wanted {value}, device reports {got}")

    if not args.quiet:
        for name, value in sequence:
            got = after.get(name, "-")
            mark = "ok " if got == str(value) else "FAIL"
            print(f"  [{mark}] {name:<30} = {got}")

    if failures:
        print("camera control lock FAILED:", file=sys.stderr)
        for item in failures:
            print(f"  {item}", file=sys.stderr)
        return 1
    if not args.quiet:
        print(f"camera controls locked on {args.device}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
