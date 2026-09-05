#!/usr/bin/env python3
"""UR5e MuJoCo viewer: gravity / PD+g / position-servo on Menagerie UR5e."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import mujoco
import mujoco.viewer
import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MJCF = ROOT / "third_party/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"
DEFAULT_SCENE = ROOT / "build" / "models" / "ur5e_menagerie_torque.mjb"

UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

Q_HOME = np.array([-np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
NV = len(UR5E_JOINTS)
START_KEYCODE = 32  # space

MODE_HELP = {
    "no-gc": "无补偿 (tau=0)，机械臂会因重力下塌",
    "hold": "hold 在 q_home",
    "push": "hold + 扰动，观察恢复",
    "compare": "依次 no-gc → hold → push（compare 固定用 gravity 控制律）",
}

LAW_HELP = {
    "gravity": "tau = g(q)",
    "pd": "tau = Kp(q_des-q) + Kd(v_des-v) + g(q)",
    "position-servo": "ctrl = q_des + g/Kp + Kd*v_des/Kp; tau = Kp(ctrl-q) - Kd*v",
}


@dataclass
class CompensatorConfig:
    kp: np.ndarray
    kd: np.ndarray


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Modes:\n"
            + "\n".join(f"  {name:8} {text}" for name, text in MODE_HELP.items())
            + "\n\nLaws (--law):\n"
            + "\n".join(f"  {name:16} {text}" for name, text in LAW_HELP.items())
        ),
    )
    parser.add_argument("--mode", choices=list(MODE_HELP), default="hold")
    parser.add_argument("--compare", action="store_true", help="shorthand for --mode compare")
    parser.add_argument(
        "--law",
        choices=list(LAW_HELP),
        default="gravity",
        help="control law for hold/push (default: gravity)",
    )
    parser.add_argument("--model", "--urdf", dest="model", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--real-time", action="store_true")
    parser.add_argument("--perturb-at", type=float, default=None)
    parser.add_argument("--perturb-duration", type=float, default=0.25)
    parser.add_argument("--perturb-torque", default="0,15,10,5,0,0")
    parser.add_argument(
        "--kp",
        default="",
        help="Comma-separated Kp [Nm/rad]; default 100 per joint (match GravityCompensator)",
    )
    parser.add_argument(
        "--kd",
        default="",
        help="Comma-separated Kd [Nm·s/rad]; default 10 per joint",
    )
    parser.add_argument(
        "--q-des",
        default="",
        help="Comma-separated q_des [rad]; default home pose",
    )
    return parser.parse_args(argv)


def apply_mode_defaults(args: argparse.Namespace) -> None:
    if args.compare:
        args.mode = "compare"

    if args.mode == "no-gc":
        if args.duration is None:
            args.duration = 3.0
        args.perturb_at = -1.0
    elif args.mode == "hold":
        if args.duration is None:
            args.duration = 5.0
        args.perturb_at = -1.0
    elif args.mode == "push":
        if args.duration is None:
            args.duration = 5.0
        if args.perturb_at is None:
            args.perturb_at = 1.0
    elif args.mode == "compare":
        args.real_time = True
        args.law = "gravity"
        if args.duration is None:
            args.duration = 8.0
        if args.perturb_at is None:
            args.perturb_at = 4.0


def parse_csv(text: str, n: int) -> np.ndarray:
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != n:
        raise ValueError(f"expected {n} comma-separated values, got {len(parts)}")
    return np.array([float(p) for p in parts], dtype=float)


def load_gains(kp_text: str, kd_text: str) -> CompensatorConfig:
    kp = parse_csv(kp_text, NV) if kp_text else np.full(NV, 100.0)
    kd = parse_csv(kd_text, NV) if kd_text else np.full(NV, 10.0)
    return CompensatorConfig(kp=kp, kd=kd)


def load_q_des(text: str) -> np.ndarray:
    return parse_csv(text, NV) if text else Q_HOME.copy()


def joint_id(model: mujoco.MjModel, name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise RuntimeError(f"MuJoCo model has no joint named '{name}'")
    return jid


def actuator_for_joint(model: mujoco.MjModel, joint_id: int) -> int:
    for act in range(model.nu):
        if (
            int(model.actuator_trntype[act]) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and int(model.actuator_trnid[act, 0]) == joint_id
        ):
            return act
    raise RuntimeError(f"MuJoCo model has no motor for joint id {joint_id}")


def build_index_maps(model: mujoco.MjModel, joint_names: list[str]):
    qpos_adr = [model.jnt_qposadr[joint_id(model, name)] for name in joint_names]
    dof_adr = [model.jnt_dofadr[joint_id(model, name)] for name in joint_names]
    act_adr = [actuator_for_joint(model, joint_id(model, name)) for name in joint_names]
    return qpos_adr, dof_adr, act_adr


def read_named(src: np.ndarray, adr: list[int]) -> np.ndarray:
    return np.array([src[a] for a in adr], dtype=float)


def write_named(dst: np.ndarray, adr: list[int], values: np.ndarray) -> None:
    for a, val in zip(adr, values):
        dst[a] = val


def gravity_torque(pin_model: pin.Model, pin_data: pin.Data, q: np.ndarray) -> np.ndarray:
    return pin.rnea(pin_model, pin_data, q, np.zeros(pin_model.nv), np.zeros(pin_model.nv))


def pd_plus_gravity(
    q: np.ndarray,
    v: np.ndarray,
    q_des: np.ndarray,
    v_des: np.ndarray,
    cfg: CompensatorConfig,
    pin_model: pin.Model,
    pin_data: pin.Data,
) -> np.ndarray:
    return (
        cfg.kp * (q_des - q) + cfg.kd * (v_des - v) + gravity_torque(pin_model, pin_data, q)
    )


def position_servo_command(
    q: np.ndarray,
    q_des: np.ndarray,
    v_des: np.ndarray,
    cfg: CompensatorConfig,
    pin_model: pin.Model,
    pin_data: pin.Data,
) -> np.ndarray:
    g = gravity_torque(pin_model, pin_data, q)
    ctrl = np.empty(NV)
    for i in range(NV):
        if cfg.kp[i] == 0.0:
            raise RuntimeError("Kp must be nonzero for position-servo feedforward")
        ctrl[i] = q_des[i] + g[i] / cfg.kp[i] + cfg.kd[i] * v_des[i] / cfg.kp[i]
    return ctrl


def position_servo_torque(
    q: np.ndarray,
    v: np.ndarray,
    q_des: np.ndarray,
    v_des: np.ndarray,
    cfg: CompensatorConfig,
    pin_model: pin.Model,
    pin_data: pin.Data,
) -> np.ndarray:
    ctrl = position_servo_command(q, q_des, v_des, cfg, pin_model, pin_data)
    return cfg.kp * (ctrl - q) - cfg.kd * v


def compute_tau(
    law: str,
    q: np.ndarray,
    v: np.ndarray,
    q_des: np.ndarray,
    v_des: np.ndarray,
    cfg: CompensatorConfig,
    pin_model: pin.Model,
    pin_data: pin.Data,
) -> np.ndarray:
    if law == "gravity":
        return gravity_torque(pin_model, pin_data, q)
    if law == "pd":
        return pd_plus_gravity(q, v, q_des, v_des, cfg, pin_model, pin_data)
    if law == "position-servo":
        return position_servo_torque(q, v, q_des, v_des, cfg, pin_model, pin_data)
    raise ValueError(f"unknown law: {law}")


def load_pinocchio_model(model_path: Path):
    if model_path.suffix.lower() in {".xml", ".mjcf"}:
        return pin.buildModelFromMJCF(str(model_path))
    return pin.buildModelFromUrdf(str(model_path))


def print_banner(args: argparse.Namespace, perturb: np.ndarray, cfg: CompensatorConfig) -> None:
    print(f"  Mode: {args.mode} — {MODE_HELP[args.mode]}")
    if args.mode != "compare":
        print(f"  Law : {args.law} — {LAW_HELP[args.law]}")
        if args.law != "gravity":
            print(f"  Kp  : {cfg.kp}")
            print(f"  Kd  : {cfg.kd}")
    if args.mode == "no-gc":
        print("  看点: 机械臂会往下塌/弯")
    elif args.mode == "hold":
        print("  看点: 机械臂应保持在 q_home")
    elif args.mode == "push":
        print(
            f"  看点: t={args.perturb_at}s 推 {args.perturb_duration}s {perturb}; "
            + ("推完应回到 home" if args.law != "gravity" else "推完仅 g(q) 不会回 home")
        )
    elif args.mode == "compare":
        print("  0~2s no-gc | 2~4s hold (g) | 4~8s push (g)")


def make_compare_controller(
    pin_model: pin.Model,
    pin_data: pin.Data,
    perturb: np.ndarray,
    perturb_at: float,
    perturb_duration: float,
    reset_state: Callable[[], None],
) -> Callable[[float, np.ndarray, np.ndarray], np.ndarray]:
    phase1_end = 2.0
    phase2_end = 4.0
    announced = {"p1": False, "p2": False, "p3": False, "p4": False}

    def control(t: float, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        if t < phase1_end:
            if not announced["p1"]:
                print("  >> [no-gc] tau = 0")
                announced["p1"] = True
            return np.zeros(NV)
        if t < phase2_end:
            if not announced["p2"]:
                reset_state()
                print("  >> [hold] tau = g(q)")
                announced["p2"] = True
            return gravity_torque(pin_model, pin_data, q)
        if not announced["p3"] and t >= perturb_at:
            print("  >> [push] 施加扰动…")
            announced["p3"] = True
        if not announced["p4"] and t >= perturb_at + perturb_duration:
            print("  >> [push] 扰动结束 (g(q) only, 不会回 home)")
            announced["p4"] = True
        tau = gravity_torque(pin_model, pin_data, q)
        if perturb_at <= t < perturb_at + perturb_duration:
            tau = tau + perturb
        return tau

    return control


def make_single_controller(
    mode: str,
    law: str,
    cfg: CompensatorConfig,
    q_des: np.ndarray,
    pin_model: pin.Model,
    pin_data: pin.Data,
    perturb: np.ndarray,
    perturb_at: float,
    perturb_duration: float,
) -> Callable[[float, np.ndarray, np.ndarray], np.ndarray]:
    v_des = np.zeros(NV)
    announced = {"push": False, "recover": False}

    def control(t: float, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        if mode == "no-gc":
            return np.zeros(NV)
        tau = compute_tau(law, q, v, q_des, v_des, cfg, pin_model, pin_data)
        if mode == "push":
            if perturb_at <= t < perturb_at + perturb_duration:
                if not announced["push"]:
                    print(f"  >> t={t:.2f}s 施加扰动 {perturb} Nm")
                    announced["push"] = True
                tau = tau + perturb
            elif t >= perturb_at + perturb_duration and not announced["recover"]:
                msg = "应回到 home" if law != "gravity" else "仅 g(q)，不会回 home"
                print(f"  >> t={t:.2f}s 扰动结束，{msg}")
                announced["recover"] = True
        return tau

    return control


def evaluate_result(
    mode: str, law: str, max_err: float, final_max_vel: float, tail_time: float
) -> bool:
    if mode == "no-gc":
        print(f"max |q - q_home| = {max_err:.6f} rad (expect large drift)")
        return max_err > 0.05
    if mode == "hold":
        print(f"max |q - q_home| = {max_err:.6f} rad")
        return max_err < 0.01
    print(f"final max |q - q_home| = {max_err:.6f} rad")
    print(f"final max |v| (last {tail_time:.2f}s) = {final_max_vel:.6f} rad/s")
    if law == "gravity":
        return final_max_vel < 0.5
    return max_err < 0.05 and final_max_vel < 0.5


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    apply_mode_defaults(args)

    if not args.model.exists():
        print(f"missing model: {args.model}", file=sys.stderr)
        print("run: ./tools/setup_menagerie.sh", file=sys.stderr)
        return 1
    if not args.scene.exists():
        print(f"missing scene: {args.scene}", file=sys.stderr)
        print("build first: cmake -S . -B build && cmake --build build", file=sys.stderr)
        return 1

    cfg = load_gains(args.kp, args.kd)
    q_des = load_q_des(args.q_des)
    perturb = parse_csv(args.perturb_torque, NV)
    perturb_at = args.perturb_at if args.perturb_at is not None else -1.0

    pin_model = load_pinocchio_model(args.model)
    pin_data = pin_model.createData()
    pin_names = [
        pin_model.names[i]
        for i in range(1, pin_model.njoints)
        if pin_model.joints[i].nq > 0
    ]
    if pin_names != UR5E_JOINTS:
        print(f"Pinocchio joints {pin_names} != expected {UR5E_JOINTS}", file=sys.stderr)
        return 1

    if args.scene.suffix == ".mjb":
        mj_model = mujoco.MjModel.from_binary_path(str(args.scene))
    else:
        mj_model = mujoco.MjModel.from_xml_path(str(args.scene))
    mj_data = mujoco.MjData(mj_model)
    qpos_adr, dof_adr, act_adr = build_index_maps(mj_model, UR5E_JOINTS)

    def reset_state() -> None:
        write_named(mj_data.qpos, qpos_adr, Q_HOME)
        write_named(mj_data.qvel, dof_adr, np.zeros(NV))
        mujoco.mj_forward(mj_model, mj_data)

    steps = max(1, int(round(args.duration / mj_model.opt.timestep)))
    max_err = 0.0
    final_max_vel = 0.0

    print("UR5e MuJoCo viewer (Menagerie)")
    print(f"  Model: {args.model}")
    print(f"  Scene: {args.scene}")
    print(f"  dt={mj_model.opt.timestep}, duration={args.duration}s")
    print_banner(args, perturb, cfg)
    print("  窗口打开后按空格开始；演示结束后请手动关闭窗口退出。")

    if args.mode == "compare":
        control = make_compare_controller(
            pin_model, pin_data, perturb, perturb_at, args.perturb_duration, reset_state
        )
    else:
        law_label = {"gravity": "g(q)", "pd": "PD+g(q)", "position-servo": "position-servo"}[
            args.law
        ]
        print(f"  >> [{args.mode}] law = {law_label}")
        control = make_single_controller(
            args.mode,
            args.law,
            cfg,
            q_des,
            pin_model,
            pin_data,
            perturb,
            perturb_at,
            args.perturb_duration,
        )

    sim_started = {"value": False}

    def on_key(keycode: int) -> None:
        if keycode == START_KEYCODE:
            sim_started["value"] = True

    with mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=on_key
    ) as viewer:
        reset_state()
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 120
        viewer.cam.elevation = -20
        print("  >> 按空格键开始仿真…")
        while viewer.is_running() and not sim_started["value"]:
            viewer.sync()
            time.sleep(0.01)
        if not viewer.is_running():
            return 0

        print("  >> 仿真开始")
        t0 = time.time()
        tail_steps = min(steps, max(1, int(round(0.1 / mj_model.opt.timestep))))
        tail_time = tail_steps * mj_model.opt.timestep
        track_err = args.mode in {"hold", "no-gc", "compare", "push"}
        track_vel = args.mode in {"push", "compare"}
        sim_finished = False
        ok = True
        i = 0

        while viewer.is_running():
            if i < steps:
                t = i * mj_model.opt.timestep
                q = read_named(mj_data.qpos, qpos_adr)
                v = read_named(mj_data.qvel, dof_adr)
                if track_err:
                    max_err = max(max_err, float(np.max(np.abs(q - Q_HOME))))
                if track_vel and i >= steps - tail_steps:
                    final_max_vel = max(final_max_vel, float(np.max(np.abs(v))))

                tau = control(t, q, v)
                for act, val in zip(act_adr, tau):
                    mj_data.ctrl[act] = val
                mujoco.mj_step(mj_model, mj_data)
                viewer.sync()

                if args.real_time:
                    target = t0 + t
                    while time.time() < target and viewer.is_running():
                        time.sleep(0.0005)
                i += 1
            else:
                if not sim_finished:
                    ok = evaluate_result(args.mode, args.law, max_err, final_max_vel, tail_time)
                    print("  >> 演示结束，请关闭窗口退出")
                    sim_finished = True
                viewer.sync()
                time.sleep(0.01)

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
