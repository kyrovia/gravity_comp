#!/usr/bin/env python3
"""Build a torque-mode MuJoCo scene from MuJoCo Menagerie UR5e."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MJCF = ROOT / "third_party/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"

DEFAULT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def build_model(
    mjcf: Path,
    damping: float,
    joints: list[str],
    armature: float | None,
) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(mjcf))
    spec.option.timestep = 0.001
    spec.option.gravity = [0.0, 0.0, -9.81]
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_RK4

    joint_set = set(joints)
    for joint in spec.joints:
        if joint.name in joint_set:
            joint.damping = [damping, 0.0, 0.0]
            if armature is not None:
                joint.armature = armature

    for act in list(spec.actuators):
        spec.delete(act)

    for name in joints:
        act = spec.add_actuator()
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.target = name
        act.set_to_motor()
        act.ctrllimited = True
        act.ctrlrange = [-180.0, 180.0]

    floor = spec.worldbody.add_geom()
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [0.0, 0.0, 0.05]
    floor.rgba = [0.2, 0.3, 0.4, 1.0]

    return spec.compile()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--armature",
        type=float,
        default=None,
        help="Optional uniform armature override. Default: keep MJCF per-joint values",
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
    if not args.mjcf.exists():
        print(f"missing Menagerie MJCF: {args.mjcf}", file=sys.stderr)
        print("run: ./tools/setup_menagerie.sh", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    model = build_model(args.mjcf, args.damping, joints, args.armature)
    mujoco.mj_saveModel(model, str(args.output))
    print(f"wrote {args.output} (nv={model.nv}, nu={model.nu})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
