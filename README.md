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

- `default` (`go2`, `go2w`)
- `dreamwaq` (`go2`, `go2w`)
- `dreamflex` (`go2`, `go2w`)
- `footstand` (`go2w` only)

Available MuJoCo scenes:

- `scene`
- `scene_flat`
- `scene_slope`
- `scene_rough`
- `scene_rough_slope`
- `scene_terrain`

## Real Robot

Usage:

```bash
./cmake_build/rl_real_go2 <go2|go2w> [config_name]
```

- DDS/network interface is auto-selected by the SDK; you no longer need to pass `eth0`.
- If `config_name` is omitted, it defaults to `default`.

Examples:

`go2` + `default`

```bash
./cmake_build/rl_real_go2 go2
```

`go2` + `dreamwaq`

```bash
./cmake_build/rl_real_go2 go2 dreamwaq
```

`go2w` + `default`

```bash
./cmake_build/rl_real_go2 go2w
```

`go2w` + `dreamwaq`

```bash
./cmake_build/rl_real_go2 go2w dreamwaq
```

`go2w` + `dreamflex`

```bash
./cmake_build/rl_real_go2 go2w dreamflex
```

`go2w` + `footstand`

```bash
./cmake_build/rl_real_go2 go2w footstand
```

## MuJoCo

Usage:

```bash
./cmake_build/rl_sim_mujoco <go2|go2w> [config_name]
```

- `scene_name` defaults to `scene`, so you no longer need to pass it for the standard simulator launch.
- If `config_name` is omitted, it defaults to `default`.
- Scene names come from `src/rl_sar_zoo/<robot>_description/mjcf/*.xml`.

Examples:

`go2` + `default`

```bash
./cmake_build/rl_sim_mujoco go2
```

`go2` + `dreamwaq`

```bash
./cmake_build/rl_sim_mujoco go2 dreamwaq
```

`go2` + `dreamflex`

```bash
./cmake_build/rl_sim_mujoco go2 dreamflex
```

Fault injection in MuJoCo:

- Available for every config, including `default`, `dreamwaq`, and `dreamflex`
- Locked joints are selected by `fault_lock_joint_offsets` (`0=hip`, `1=thigh`, `2=calf`); `go2/dreamflex` and `go2w/dreamflex` use `[2]`, so only the selected leg's calf is folded/held
- Locked targets use `fault_lock_hip_q` / `fault_lock_thigh_q` / `fault_lock_calf_q` from the active config, falling back to `policy/<robot>/base.yaml` defaults when not overridden
- Switching the fault leg is sequential: the old leg is released first, then after the release ramp and a short settle delay the new leg is locked
- If those keys are absent, the simulator falls back to the joint's default stand angle
- Keyboard:
  - `T`: cycle `none -> locked -> none`
  - `Y` / `U`: select fault leg `- / +`
- Gamepad:
  - `LB + A`: cycle fault mode
  - `LB + DPad Left/Right`: select fault leg `- / +`

Fault injection on hardware (`rl_real_go2`):

- Uses the same controls as MuJoCo
- `LB + A`: cycle `none -> locked -> none`
- `LB + DPad Left/Right`: select fault leg `- / +`
- Fault-leg switching uses the same sequential release-then-lock behavior as MuJoCo
- Locked mode holds the joints selected by `fault_lock_joint_offsets`; `go2/dreamflex` locks calf only at `fault_lock_calf_q`
- `go2w/dreamflex` uses the same calf-only lock behavior and ignores wheels in the policy fault vector

`go2w` + `default`

```bash
./cmake_build/rl_sim_mujoco go2w
```

`go2w` + `dreamwaq`

```bash
./cmake_build/rl_sim_mujoco go2w dreamwaq
```

`go2w` + `dreamflex`

```bash
./cmake_build/rl_sim_mujoco go2w dreamflex
```

`go2w` + `footstand`

```bash
./cmake_build/rl_sim_mujoco go2w footstand
```

Terrain examples:

```bash
./cmake_build/rl_sim_mujoco go2 scene_slope dreamflex
```

```bash
./cmake_build/rl_sim_mujoco go2 scene_rough dreamflex
```

```bash
./cmake_build/rl_sim_mujoco go2 scene_rough_slope dreamflex
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
  - `go2w + dreamflex` -> `policy/go2w/dreamflex/`
  - `go2w + footstand` -> `policy/go2w/footstand/`

## Notes

- MuJoCo joystick discovery scans `/dev/input/js0` through `/dev/input/js9`.
- Policies kept in this repo are only under `policy/go2` and `policy/go2w`.
- `src/rl_sar_zoo` should be committed with this repository and now keeps only the MuJoCo `mjcf` assets required by `go2_description` and `go2w_description`.
