#!/usr/bin/env python3
"""Build a MuJoCo binary scene matched to a URDF."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco

DEFAULT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]  # Keep in sync with models/ur5e.hpp.


def build_model(
    urdf: Path,
    armature: float,
    damping: float,
    joints: list[str],
) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(urdf))
    spec.option.timestep = 0.001
    spec.option.gravity = [0.0, 0.0, -9.81]
    joint_set = set(joints)
    for joint in spec.joints:
        if joint.name in joint_set:
            joint.damping = [damping, 0.0, 0.0]
            joint.armature = armature
    for name in joints:
        act = spec.add_actuator()
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.target = name
        act.set_to_motor()
        act.ctrllimited = True
        act.ctrlrange = [-180.0, 180.0]
    return spec.compile()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--armature",
        type=float,
        default=0.1,
        help="Rotor inertia; must match PinModel's armature argument",
    )
    parser.add_argument("--damping", type=float, default=1.0)
    parser.add_argument(
        "--joints",
        default=",".join(DEFAULT_JOINTS),
        help="Comma-separated 1-DoF joint names to actuate",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    joints = [name.strip() for name in args.joints.split(",") if name.strip()]
    if not joints:
        print("at least one joint name is required", file=sys.stderr)
        return 1
    if not args.urdf.exists():
        print(f"missing URDF: {args.urdf}", file=sys.stderr)
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    model = build_model(args.urdf, args.armature, args.damping, joints)
    mujoco.mj_saveModel(model, str(args.output))
    print(f"wrote {args.output} (nv={model.nv}, nu={model.nu})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
