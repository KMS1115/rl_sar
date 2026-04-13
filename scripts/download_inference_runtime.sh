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
ARCHIVE_DIR="${MODEL_INTERFACE_DIR}/archives"
mkdir -p "${MODEL_INTERFACE_DIR}"
mkdir -p "${ARCHIVE_DIR}"

ONNXRUNTIME_VERSION="1.22.0"

resolve_onnxruntime_package() {
    case "${OS_TYPE}" in
        Linux)
            if [ "${ARCH_TYPE}" = "aarch64" ]; then
                ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime-aarch64"
                ONNXRUNTIME_ARCHIVE_NAME="onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz"
            else
                ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime-x64"
                ONNXRUNTIME_ARCHIVE_NAME="onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz"
            fi
            ;;
        Darwin)
            if [ "${ARCH_TYPE}" = "arm64" ]; then
                ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime-macos-arm64"
                ONNXRUNTIME_ARCHIVE_NAME="onnxruntime-osx-arm64-${ONNXRUNTIME_VERSION}.tgz"
            else
                ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime-macos-x64"
                ONNXRUNTIME_ARCHIVE_NAME="onnxruntime-osx-x86_64-${ONNXRUNTIME_VERSION}.tgz"
            fi
            ;;
        MINGW*|MSYS*)
            ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime"
            ONNXRUNTIME_ARCHIVE_NAME="onnxruntime-win-x64-${ONNXRUNTIME_VERSION}.zip"
            ;;
        *)
            ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime"
            ONNXRUNTIME_ARCHIVE_NAME=""
            ;;
    esac

    if [ -n "${ONNXRUNTIME_ARCHIVE_NAME}" ]; then
        ONNXRUNTIME_DOWNLOAD_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ONNXRUNTIME_ARCHIVE_NAME}"
    else
        ONNXRUNTIME_DOWNLOAD_URL=""
    fi
}

is_onnxruntime_valid() {
    if [ ! -d "$ONNXRUNTIME_DIR" ]; then
        return 1
    fi

    if [ ! -d "$ONNXRUNTIME_DIR/lib" ] || [ ! -d "$ONNXRUNTIME_DIR/include" ]; then
        return 1
    fi

    return 0
}

extract_onnxruntime_archive() {
    local archive_path="$1"

    print_info "Extracting ONNX Runtime from ${archive_path}..."

    local temp_dir="${MODEL_INTERFACE_DIR}/temp_extract"
    rm -rf "${temp_dir}"
    mkdir -p "${temp_dir}"

    if [[ "${archive_path}" == *.tgz ]]; then
        tar -xzf "${archive_path}" -C "${temp_dir}" || {
            print_error "Extraction failed"
            rm -rf "${temp_dir}"
            exit 1
        }
    else
        unzip -o -q "${archive_path}" -d "${temp_dir}" || {
            print_error "Extraction failed"
            rm -rf "${temp_dir}"
            exit 1
        }
    fi

    local extracted_dir
    extracted_dir=$(find "${temp_dir}" -maxdepth 1 -type d -name "onnxruntime-*" | head -n 1)
    if [ -z "${extracted_dir}" ] || [ ! -d "${extracted_dir}" ]; then
        print_error "Incorrect directory structure after extraction"
        rm -rf "${temp_dir}"
        exit 1
    fi

    rm -rf "${ONNXRUNTIME_DIR}"
    mv "${extracted_dir}" "${ONNXRUNTIME_DIR}"

    rm -rf "${temp_dir}"

    print_success "ONNX Runtime ${ONNXRUNTIME_VERSION} installed successfully"
}

install_onnxruntime_from_local_archive() {
    local archive_path=""
    local legacy_archive_path="${MODEL_INTERFACE_DIR}/${ONNXRUNTIME_ARCHIVE_NAME}"
    local cached_archive_path="${ARCHIVE_DIR}/${ONNXRUNTIME_ARCHIVE_NAME}"

    if [ -f "${cached_archive_path}" ]; then
        archive_path="${cached_archive_path}"
    elif [ -f "${legacy_archive_path}" ]; then
        archive_path="${legacy_archive_path}"
    else
        return 1
    fi

    print_info "Found cached ONNX Runtime archive: ${archive_path}"
    extract_onnxruntime_archive "${archive_path}"
    return 0
}

download_onnxruntime() {
    local archive_path="${ARCHIVE_DIR}/${ONNXRUNTIME_ARCHIVE_NAME}"

    if [ -z "${ONNXRUNTIME_DOWNLOAD_URL}" ]; then
        print_error "Unsupported OS: ${OS_TYPE}"
        exit 1
    fi

    print_info "Downloading ONNX Runtime ${ONNXRUNTIME_VERSION}..."
    print_info "Platform: ${OS_TYPE} (${ARCH_TYPE})"
    print_info "URL: ${ONNXRUNTIME_DOWNLOAD_URL}"

    if command -v curl &> /dev/null; then
        curl -L --progress-bar -o "${archive_path}" "${ONNXRUNTIME_DOWNLOAD_URL}" || {
            print_error "Download failed"
            rm -f "${archive_path}"
            exit 1
        }
    elif command -v wget &> /dev/null; then
        wget --show-progress -O "${archive_path}" "${ONNXRUNTIME_DOWNLOAD_URL}" || {
            print_error "Download failed"
            rm -f "${archive_path}"
            exit 1
        }
    else
        print_error "curl or wget is required to download files"
        exit 1
    fi

    extract_onnxruntime_archive "${archive_path}"
}

print_header "[ONNX Runtime Setup]"
resolve_onnxruntime_package

if is_onnxruntime_valid; then
    print_success "ONNX Runtime already exists and is valid"
    print_info "Location: ${ONNXRUNTIME_DIR}"
    exit 0
fi

if [ -d "${ONNXRUNTIME_DIR}" ]; then
    print_warning "ONNX Runtime directory incomplete, re-downloading..."
    rm -rf "${ONNXRUNTIME_DIR}"
fi

if ! install_onnxruntime_from_local_archive; then
    print_warning "Cached ONNX Runtime archive not found in ${ARCHIVE_DIR}"
    download_onnxruntime
fi

if ! is_onnxruntime_valid; then
    print_error "ONNX Runtime installation failed"
    exit 1
fi

print_success "ONNX Runtime setup completed!"
print_info "Location: ${ONNXRUNTIME_DIR}"
