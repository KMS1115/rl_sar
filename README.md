# camel_sar

Standalone `go2` / `go2w` runtime for:

- MuJoCo simulation
- Unitree low-level hardware control
- ONNX policy inference

The MuJoCo simulator now always loads one fixed terrain course: `scene.xml`.
That course runs in this order: 10 degree uphill slope, 10 degree downhill slope, heightfield gravel floor, 15 cm stair climb, then 15 cm stair descent.
The course sits on an infinite ground plane, so each obstacle can be bypassed if the robot cannot clear it.

## Build

MuJoCo simulation:

```bash
./build.sh -mj
```

Real robot:

```bash
./build.sh -m
```

Build outputs:

```bash
./cmake_build/rl_sim_mujoco
./cmake_build/rl_real_go2
```

`build.sh` uses the vendored `src/rl_sar_zoo` descriptions and the cached runtime archives under `library/inference_runtime/archives/`.

## Simulation

Usage:

```bash
./cmake_build/rl_sim_mujoco <go2|go2w> [config_name]
```

Examples:

```bash
./cmake_build/rl_sim_mujoco go2 dreamflex
./cmake_build/rl_sim_mujoco go2w dreamwaq
```

Available configs:

- `go2`: `default`, `dreamwaq`, `dreamflex`
- `go2w`: `default`, `dreamwaq`, `dreamflex`, `footstand`

Scene arguments are ignored for compatibility with old commands. The simulator always opens:

```text
src/rl_sar_zoo/<robot>_description/mjcf/scene.xml
```

## Fault Injection

Fault injection is limited to the two rear legs:

- `2`: RR
- `3`: RL

Controls:

- Keyboard: `T` toggles fault mode, `Y` / `U` selects RR/RL
- Gamepad: `LB + A` toggles fault mode, `LB + DPad Left/Right` selects RR/RL

Joint locking is configured by:

- `fault_allowed_leg_indices`: allowed leg IDs, currently `[2, 3]`
- `fault_lock_joint_offsets`: locked joints on the selected leg, where `0=hip`, `1=thigh`, `2=calf`
- `fault_lock_hip_q`, `fault_lock_thigh_q`, `fault_lock_calf_q`: locked targets

## Real Robot

Usage:

```bash
./cmake_build/rl_real_go2 <go2|go2w> [config_name]
```

Examples:

```bash
./cmake_build/rl_real_go2 go2 dreamwaq
./cmake_build/rl_real_go2 go2w dreamflex
```

Hardware fault controls use the same RR/RL limit and gamepad controls as MuJoCo.

## Policy Lookup

Runtime loading order:

1. `policy/<robot>/base.yaml`
2. `policy/<robot>/<config_name>/config.yaml`
3. `policy/<robot>/<config_name>/policy.onnx`

Examples:

- `go2 dreamflex` -> `policy/go2/dreamflex/`
- `go2w dreamwaq` -> `policy/go2w/dreamwaq/`

## Notes

- MuJoCo joystick discovery scans `/dev/input/js0` through `/dev/input/js9`.
- The only MuJoCo map files kept are `go2_description/mjcf/scene.xml` and `go2w_description/mjcf/scene.xml`.
