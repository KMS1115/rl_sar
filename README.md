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

## Run

Real robot:

```bash
./cmake_build/bin/rl_real_go2 <NETWORK_INTERFACE> [wheel]
```

- no `wheel`: `go2`
- `wheel`: `go2w`

MuJoCo:

```bash
./cmake_build/bin/rl_sim_mujoco <go2|go2w> <scene_name>
```

Example:

```bash
./cmake_build/bin/rl_sim_mujoco go2 scene
```

## Notes

- MuJoCo joystick discovery scans `/dev/input/js0` through `/dev/input/js9`.
- Policies kept in this repo are only under [`policy/go2`](/home/ms/rl_sar/policy/go2) and [`policy/go2w`](/home/ms/rl_sar/policy/go2w).
- `src/rl_sar_zoo` should be committed with this repository and now keeps only the MuJoCo `mjcf` assets required by `go2_description` and `go2w_description`.
