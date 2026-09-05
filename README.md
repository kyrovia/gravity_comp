# gravity_comp

从 `mujoco-robotics-lab` 抽出的重力补偿库（C++17）。工程主链路与常见力矩控制一致：

```
URDF/MJCF  →  Pinocchio  →  RNEA(q, 0, 0) = g(q)  →  τ_ff  →  关节力矩
```

- **动力学**：`PinModel::gravity(q)` 调用 `rnea(q, 0, 0)`，即零速度、零加速度下的逆动力学
- **控制层**：`GravityCompensator` 把 \(g(q)\) 作为力矩前馈（可选 PD）
- **仿真**（可选）：MuJoCo 力矩模式，用来验证前馈是否站得住

核心库 `gct` 只依赖 Eigen + Pinocchio。MuJoCo 验证在 `gct_mujoco` 里，默认不构建；需要时使用 `-DGCT_ENABLE_MUJOCO=ON`。

## 依赖

| 组件 | 用途 |
|---|---|
| Eigen3 | 向量/矩阵 |
| Pinocchio | URDF 建模，RNEA 算 \(g(q)\) |
| MuJoCo 3.x | UR5e 仿真（可选） |
| Catch2 | 测试 |

### Conda 环境

需要先激活环境。CMake 读 `CONDA_PREFIX`，不再使用仓库里的 `.deps`。

```bash
conda create -y -n gct -c conda-forge pinocchio eigen catch2 cxx-compiler
conda activate gct
```

`cxx-compiler` 用来和 conda 的 libstdc++ 对齐。MuJoCo 可以装在同一个环境里，或另外指定路径。

ROS 2 或其它 prefix：

```bash
cmake -S . -B build -DGCT_PINOCCHIO_ROOT=/opt/ros/humble
```

### MuJoCo

任选其一：

- 当前已激活环境里装了 MuJoCo wheel（读 `CONDA_PREFIX`）
- `-DGCT_MUJOCO_ROOT=/path/to/mujoco`（含 `libmujoco` 和 `include/mujoco`）
- 系统路径里能被 `find_library(mujoco)` 找到

启用 MuJoCo 仿真：

```bash
cmake -S . -B build -DGCT_ENABLE_MUJOCO=ON
```

## 构建

```bash
conda activate gct
cmake -S . -B build -DGCT_ENABLE_MUJOCO=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

匹配的 MuJoCo 场景生成到 `build/models/ur5e_menagerie_torque.mjb`（来自 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) 的 `universal_robots_ur5e`）。首次构建前：

```bash
./tools/setup_menagerie.sh
```

## 示例

```bash
# 场景模式 (--mode) × 控制律 (--law)
python examples/ur5e_mujoco_viewer.py --mode hold --law gravity --real-time
python examples/ur5e_mujoco_viewer.py --mode push --law pd --real-time
python examples/ur5e_mujoco_viewer.py --mode push --law position-servo --real-time
python examples/ur5e_mujoco_viewer.py --mode compare --real-time   # 固定 gravity 三阶段

# 可选增益（默认 Kp=100, Kd=10，与 GravityCompensator 一致）
python examples/ur5e_mujoco_viewer.py --mode push --law pd --kp 80,80,80,80,80,80 --kd 8,8,8,8,8,8 --real-time

# CMake: ur5e_mujoco_viewer / _no_gc / _hold / _push
#        ur5e_mujoco_viewer_pd / ur5e_mujoco_viewer_servo
```

仓库只保留 MuJoCo 可视化示例。可用 `--urdf` / `--scene` 覆盖默认模型与场景路径。

## API

```cpp
#include "gct/gravity_compensator.hpp"
#include "gct/pin_model.hpp"

gct::PinModel pin("/path/to/robot.urdf");
gct::GravityCompensator gc(pin);

gct::VectorXd tau = gc.gravity_torque(q);      // τ = RNEA(q, 0, 0)
tau = gc.pd_plus_gravity(q, v, q_des, v_des);  // PD + g(q)
```

仿真验证（链接 `gct_mujoco`）：

```cpp
#include "gct_mujoco/arm_simulator.hpp"
#include "gct_mujoco/gravity_validation.hpp"

gct::mujoco::ArmSimulator simulator("/path/to/scene.mjb",
                                    pin.joint_names());
gct::mujoco::simulate_gravity_hold(simulator, gc, q);
gct::mujoco::compare_gravity(pin, simulator, q);
```

## 模型

- `third_party/mujoco_menagerie/universal_robots_ur5e/` — 官方 MuJoCo Menagerie UR5e（网格 + 动力学）
- `tests/fixtures/ur5e_fixture.hpp` — 仅测试使用的关节名与 home 配置
- 构建时由 `tools/build_menagerie_torque_scene.py` 生成力矩模式场景

## License

Apache-2.0
