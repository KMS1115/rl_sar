Place cached ONNX Runtime release archives here for offline builds.

Expected filenames for the current setup:

- `onnxruntime-linux-aarch64-1.22.0.tgz`
- `onnxruntime-linux-x64-1.22.0.tgz`
- `onnxruntime-osx-arm64-1.22.0.tgz`
- `onnxruntime-osx-x86_64-1.22.0.tgz`
- `onnxruntime-win-x64-1.22.0.zip`

`build.sh` and `scripts/download_inference_runtime.sh` will extract the archive
that matches the current platform into `library/inference_runtime/`.
