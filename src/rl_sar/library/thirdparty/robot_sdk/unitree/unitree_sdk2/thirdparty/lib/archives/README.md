Place Linux-created Unitree SDK third-party runtime archives here for offline builds.

Expected filenames:

- `unitree-sdk2-thirdparty-aarch64.tgz`
- `unitree-sdk2-thirdparty-x86_64.tgz`

Each archive should expand to exactly one top-level directory:

- `aarch64/`
- `x86_64/`

`build.sh` restores the matching directory into `thirdparty/lib/` before CMake runs.
