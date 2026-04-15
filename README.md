# rl_sar

Trimmed standalone build focused on:

- `go2`
- `go2w`
- standalone CMake build
- MuJoCo simulation

Removed from this tree:

- ROS 1 / ROS 2 packages
- Gazebo launch/world assets
- all non-`go2` / non-`go2w` robot code and policies

## Build

Real robot binary:

```bash
./build.sh -m
```

MuJoCo simulation binary:

```bash
./build.sh -mj
```

`./build.sh -mj` uses the bundled `src/rl_sar_zoo` tree in this repository.
The build no longer clones robot descriptions on demand.

Built binaries are placed at:

```bash
./cmake_build/rl_real_go2
./cmake_build/rl_sim_mujoco
```

Offline ONNX Runtime setup:

- Keep the official ONNX Runtime archives under `library/inference_runtime/archives/`.
- Linux examples:
  - `onnxruntime-linux-aarch64-1.22.0.tgz`
  - `onnxruntime-linux-x64-1.22.0.tgz`
- `build.sh` extracts the matching archive automatically before CMake runs.
- Do not commit expanded `library/inference_runtime/onnxruntime-*` directories anymore.

Offline Unitree SDK third-party runtime setup:

- Keep the cached DDS runtime archives under `src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2/thirdparty/lib/archives/`.
- Linux examples:
  - `unitree-sdk2-thirdparty-aarch64.tgz`
  - `unitree-sdk2-thirdparty-x86_64.tgz`
- `build.sh` restores the matching `thirdparty/lib/<arch>/` tree automatically before CMake runs.
- Do not commit expanded `thirdparty/lib/aarch64` or `thirdparty/lib/x86_64` directories anymore.

## Run

Available robots:

- `go2`
- `go2w`

Available controller configs:

- `default`
- `dreamwaq`
- `dreamflex`

Available MuJoCo scenes:

- `scene`
- `scene_terrain`

## Real Robot

Usage:

```bash
./cmake_build/rl_real_go2 <NETWORK_INTERFACE> [wheel] [config_name]
```

- If `wheel` is omitted, the robot is `go2`.
- If `wheel` is present, the robot is `go2w`.
- If `config_name` is omitted, it defaults to `default`.
- Argument order matters for `go2w`: put `wheel` before `config_name`.

Examples:

`go2` + `default`

```bash
./cmake_build/rl_real_go2 eth0
```

`go2` + `dreamwaq`

```bash
./cmake_build/rl_real_go2 eth0 dreamwaq
```

`go2w` + `default`

```bash
./cmake_build/rl_real_go2 eth0 wheel
```

`go2w` + `dreamwaq`

```bash
./cmake_build/rl_real_go2 eth0 wheel dreamwaq
```

## MuJoCo

Usage:

```bash
./cmake_build/rl_sim_mujoco <go2|go2w> <scene_name> [config_name]
```

- If `config_name` is omitted, it defaults to `default`.
- Scene names come from `src/rl_sar_zoo/<robot>_description/mjcf/*.xml`.

Examples:

`go2` + `default`

```bash
./cmake_build/rl_sim_mujoco go2 scene
```

`go2` + `dreamwaq`

```bash
./cmake_build/rl_sim_mujoco go2 scene dreamwaq
```

`go2` + `dreamflex`

```bash
./cmake_build/rl_sim_mujoco go2 scene dreamflex
```

Fault injection in MuJoCo:

- Available for every config, including `default`, `dreamwaq`, and `dreamflex`
- Locked `thigh` / `calf` joints use fixed fold targets from `fault_lock_thigh_q` / `fault_lock_calf_q` in `policy/<robot>/base.yaml`
- If those keys are absent, the simulator falls back to the joint's `seated_dof_pos`
- Keyboard:
  - `T`: cycle `none -> locked -> weakened -> none`
  - `Y` / `U`: select fault joint `- / +`
  - `I` / `O`: adjust fault severity `- / +`
- Gamepad:
  - `LB + A`: cycle fault mode
  - `LB + DPad Left/Right`: select fault joint `- / +`
  - `LB + DPad Down/Up`: adjust fault severity `- / +`

Fault injection on hardware (`rl_real_go2`):

- Uses the same controls as MuJoCo
- `LB + A`: cycle `none -> locked -> weakened -> none`
- `LB + DPad Left/Right`: select fault joint `- / +`
- `LB + DPad Down/Up`: adjust severity
- Locked mode holds the selected joint at the current joint angle, unless `fault_lock_thigh_q` / `fault_lock_calf_q` is configured in `policy/<robot>/base.yaml`

`go2w` + `default`

```bash
./cmake_build/rl_sim_mujoco go2w scene
```

`go2w` + `dreamwaq`

```bash
./cmake_build/rl_sim_mujoco go2w scene dreamwaq
```

Terrain examples:

```bash
./cmake_build/rl_sim_mujoco go2 scene_terrain
```

```bash
./cmake_build/rl_sim_mujoco go2w scene_terrain dreamwaq
```

Policy/config lookup:

- The runtime always loads `policy/<robot_name>/base.yaml` first.
- Then it loads `policy/<robot_name>/<config_name>/config.yaml`.
- Then it loads `policy/<robot_name>/<config_name>/policy.onnx`.
- Examples:
  - `go2 + default` -> `policy/go2/default/`
  - `go2 + dreamwaq` -> `policy/go2/dreamwaq/`
  - `go2 + dreamflex` -> `policy/go2/dreamflex/`
  - `go2w + default` -> `policy/go2w/default/`
  - `go2w + dreamwaq` -> `policy/go2w/dreamwaq/`

## Notes

- MuJoCo joystick discovery scans `/dev/input/js0` through `/dev/input/js9`.
- Policies kept in this repo are only under `policy/go2` and `policy/go2w`.
- `src/rl_sar_zoo` should be committed with this repository and now keeps only the MuJoCo `mjcf` assets required by `go2_description` and `go2w_description`.
