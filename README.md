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

Build standalone binaries:

```bash
./build.sh -m
```

Build standalone binaries with MuJoCo support:

```bash
./build.sh -mj
```

`./build.sh -mj` uses the bundled `src/rl_sar_zoo` tree in this repository.
The build no longer clones robot descriptions on demand.

Offline ONNX Runtime setup:

- Keep the official ONNX Runtime archives under `library/inference_runtime/archives/`.
- Linux examples:
  - `onnxruntime-linux-aarch64-1.22.0.tgz`
  - `onnxruntime-linux-x64-1.22.0.tgz`
- `build.sh` extracts the matching archive automatically before CMake runs.
- Do not commit expanded `library/inference_runtime/onnxruntime-*` directories anymore.

## Run

Real robot:

```bash
./cmake_build/bin/rl_real_go2 <NETWORK_INTERFACE> [wheel] [config_name]
```

- no `wheel`: `go2`
- `wheel`: `go2w`
- default `config_name`: `default`
- available examples:
  - `go2/default`
  - `go2/dreamwaq`
  - `go2w/default`
  - `go2w/dreamwaq`

MuJoCo:

```bash
./cmake_build/bin/rl_sim_mujoco <go2|go2w> <scene_name> [config_name]
```

Example:

```bash
./cmake_build/bin/rl_sim_mujoco go2 scene
```

```bash
./cmake_build/bin/rl_sim_mujoco go2 scene dreamwaq
```

```bash
./cmake_build/bin/rl_sim_mujoco go2w scene
```

Real-robot examples:

```bash
./cmake_build/bin/rl_real_go2 eth0
```

```bash
./cmake_build/bin/rl_real_go2 eth0 dreamwaq
```

```bash
./cmake_build/bin/rl_real_go2 eth0 wheel
```

Policy/config lookup:

- The runtime always loads `policy/<robot_name>/base.yaml` first.
- Then it loads `policy/<robot_name>/<config_name>/config.yaml`.
- Then it loads `policy/<robot_name>/<config_name>/policy.onnx`.
- Examples:
  - `go2 + default` -> `policy/go2/default/`
  - `go2 + dreamwaq` -> `policy/go2/dreamwaq/`
  - `go2w + default` -> `policy/go2w/default/`
  - `go2w + dreamwaq` -> `policy/go2w/dreamwaq/`

## Notes

- MuJoCo joystick discovery scans `/dev/input/js0` through `/dev/input/js9`.
- Policies kept in this repo are only under [`policy/go2`](/home/ms/rl_sar/policy/go2) and [`policy/go2w`](/home/ms/rl_sar/policy/go2w).
- `src/rl_sar_zoo` should be committed with this repository and now keeps only the MuJoCo `mjcf` assets required by `go2_description` and `go2w_description`.
