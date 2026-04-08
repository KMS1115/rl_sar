#!/bin/bash

# ONNX Runtime setup script
# Usage: ./download_inference_runtime.sh [target_dir]
#   target_dir: Target directory (default: library/inference_runtime)

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

source "${SCRIPT_DIR}/common.sh"

OS_TYPE="$(uname -s)"
ARCH_TYPE="$(uname -m)"

if [ $# -eq 0 ]; then
    TARGET_DIR="library/inference_runtime"
else
    TARGET_DIR="$1"
fi

MODEL_INTERFACE_DIR="${PROJECT_ROOT}/${TARGET_DIR}"
mkdir -p "${MODEL_INTERFACE_DIR}"

ONNXRUNTIME_VERSION="1.22.0"

case "${OS_TYPE}" in
    Linux)
        if [ "${ARCH_TYPE}" = "aarch64" ]; then
            ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime-aarch64"
        else
            ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime-x64"
        fi
        ;;
    Darwin)
        if [ "${ARCH_TYPE}" = "arm64" ]; then
            ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime-macos-arm64"
        else
            ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime-macos-x64"
        fi
        ;;
    *)
        ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime"
        ;;
esac

is_onnxruntime_valid() {
    if [ ! -d "$ONNXRUNTIME_DIR" ]; then
        return 1
    fi

    if [ ! -d "$ONNXRUNTIME_DIR/lib" ] || [ ! -d "$ONNXRUNTIME_DIR/include" ]; then
        return 1
    fi

    return 0
}

download_onnxruntime() {
    local url=""
    local archive_name=""

    case "${OS_TYPE}" in
        Darwin)
            if [ "${ARCH_TYPE}" = "arm64" ]; then
                url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-osx-arm64-${ONNXRUNTIME_VERSION}.tgz"
                archive_name="onnxruntime-osx-arm64-${ONNXRUNTIME_VERSION}.tgz"
            else
                url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-osx-x86_64-${ONNXRUNTIME_VERSION}.tgz"
                archive_name="onnxruntime-osx-x86_64-${ONNXRUNTIME_VERSION}.tgz"
            fi
            ;;
        Linux)
            if [ "${ARCH_TYPE}" = "aarch64" ]; then
                url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz"
                archive_name="onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz"
            else
                url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz"
                archive_name="onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz"
            fi
            ;;
        MINGW*|MSYS*)
            url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-win-x64-${ONNXRUNTIME_VERSION}.zip"
            archive_name="onnxruntime-win-x64-${ONNXRUNTIME_VERSION}.zip"
            ;;
        *)
            print_error "Unsupported OS: ${OS_TYPE}"
            exit 1
            ;;
    esac

    local archive_path="${MODEL_INTERFACE_DIR}/${archive_name}"

    print_info "Downloading ONNX Runtime ${ONNXRUNTIME_VERSION}..."
    print_info "Platform: ${OS_TYPE} (${ARCH_TYPE})"
    print_info "URL: ${url}"

    if command -v curl &> /dev/null; then
        curl -L --progress-bar -o "${archive_path}" "${url}" || {
            print_error "Download failed"
            rm -f "${archive_path}"
            exit 1
        }
    elif command -v wget &> /dev/null; then
        wget --show-progress -O "${archive_path}" "${url}" || {
            print_error "Download failed"
            rm -f "${archive_path}"
            exit 1
        }
    else
        print_error "curl or wget is required to download files"
        exit 1
    fi

    print_info "Extracting ONNX Runtime..."
    local temp_dir="${MODEL_INTERFACE_DIR}/temp_extract"
    rm -rf "${temp_dir}"
    mkdir -p "${temp_dir}"

    if [[ "$archive_name" == *.tgz ]]; then
        tar -xzf "${archive_path}" -C "${temp_dir}" || {
            print_error "Extraction failed"
            rm -rf "${temp_dir}" "${archive_path}"
            exit 1
        }
    else
        unzip -o -q "${archive_path}" -d "${temp_dir}" || {
            print_error "Extraction failed"
            rm -rf "${temp_dir}" "${archive_path}"
            exit 1
        }
    fi

    local extracted_dir
    extracted_dir=$(find "${temp_dir}" -maxdepth 1 -type d -name "onnxruntime-*" | head -n 1)
    if [ -z "${extracted_dir}" ] || [ ! -d "${extracted_dir}" ]; then
        print_error "Incorrect directory structure after extraction"
        rm -rf "${temp_dir}" "${archive_path}"
        exit 1
    fi

    rm -rf "${ONNXRUNTIME_DIR}"
    mv "${extracted_dir}" "${ONNXRUNTIME_DIR}"

    rm -rf "${temp_dir}" "${archive_path}"

    print_success "ONNX Runtime ${ONNXRUNTIME_VERSION} installed successfully"
}

print_header "[ONNX Runtime Setup]"

if is_onnxruntime_valid; then
    print_success "ONNX Runtime already exists and is valid"
    print_info "Location: ${ONNXRUNTIME_DIR}"
    exit 0
fi

if [ -d "${ONNXRUNTIME_DIR}" ]; then
    print_warning "ONNX Runtime directory incomplete, re-downloading..."
    rm -rf "${ONNXRUNTIME_DIR}"
fi

download_onnxruntime

if ! is_onnxruntime_valid; then
    print_error "ONNX Runtime installation failed"
    exit 1
fi

print_success "ONNX Runtime setup completed!"
print_info "Location: ${ONNXRUNTIME_DIR}"
