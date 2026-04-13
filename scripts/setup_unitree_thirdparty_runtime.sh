#!/bin/bash

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

source "${SCRIPT_DIR}/common.sh"

OS_TYPE="$(uname -s)"
ARCH_TYPE="$(uname -m)"

LIB_ROOT="${PROJECT_ROOT}/src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2/thirdparty/lib"
ARCHIVE_DIR="${LIB_ROOT}/archives"

resolve_archive_name() {
    case "${OS_TYPE}" in
        Linux)
            if [ "${ARCH_TYPE}" = "aarch64" ]; then
                TARGET_ARCH_DIR="${LIB_ROOT}/aarch64"
                ARCHIVE_NAME="unitree-sdk2-thirdparty-aarch64.tgz"
            else
                TARGET_ARCH_DIR="${LIB_ROOT}/x86_64"
                ARCHIVE_NAME="unitree-sdk2-thirdparty-x86_64.tgz"
            fi
            ;;
        *)
            print_error "Unsupported OS for Unitree third-party runtime: ${OS_TYPE}"
            exit 1
            ;;
    esac
}

is_target_valid() {
    if [ ! -d "${TARGET_ARCH_DIR}" ]; then
        return 1
    fi

    if [ ! -f "${TARGET_ARCH_DIR}/libddsc.so" ] || [ ! -f "${TARGET_ARCH_DIR}/libddscxx.so" ]; then
        return 1
    fi

    if [ ! -L "${TARGET_ARCH_DIR}/libddsc.so.0" ] || [ ! -L "${TARGET_ARCH_DIR}/libddscxx.so.0" ]; then
        return 1
    fi

    if [ "$(readlink "${TARGET_ARCH_DIR}/libddsc.so.0")" != "libddsc.so" ]; then
        return 1
    fi

    if [ "$(readlink "${TARGET_ARCH_DIR}/libddscxx.so.0")" != "libddscxx.so" ]; then
        return 1
    fi

    return 0
}

restore_from_archive() {
    local archive_path="${ARCHIVE_DIR}/${ARCHIVE_NAME}"

    if [ ! -f "${archive_path}" ]; then
        print_error "Missing cached Unitree runtime archive: ${archive_path}"
        print_error "Restore ${TARGET_ARCH_DIR} from a Linux-created archive before building."
        exit 1
    fi

    print_info "Restoring Unitree third-party runtime from ${archive_path}"

    local temp_dir="${LIB_ROOT}/temp_extract"
    rm -rf "${temp_dir}"
    mkdir -p "${temp_dir}"

    tar -xzf "${archive_path}" -C "${temp_dir}" || {
        print_error "Extraction failed"
        rm -rf "${temp_dir}"
        exit 1
    }

    local extracted_dir
    extracted_dir=$(find "${temp_dir}" -mindepth 1 -maxdepth 1 -type d | head -n 1)
    if [ -z "${extracted_dir}" ] || [ ! -d "${extracted_dir}" ]; then
        print_error "Incorrect directory structure after extraction"
        rm -rf "${temp_dir}"
        exit 1
    fi

    rm -rf "${TARGET_ARCH_DIR}"
    mv "${extracted_dir}" "${TARGET_ARCH_DIR}"
    rm -rf "${temp_dir}"

    print_success "Unitree third-party runtime restored"
}

print_header "[Unitree Third-Party Runtime Setup]"
mkdir -p "${ARCHIVE_DIR}"
resolve_archive_name

if is_target_valid; then
    print_success "Unitree third-party runtime already exists and is valid"
    print_info "Location: ${TARGET_ARCH_DIR}"
    exit 0
fi

print_warning "Unitree third-party runtime missing or invalid for ${ARCH_TYPE}"
restore_from_archive

if ! is_target_valid; then
    print_error "Unitree third-party runtime setup failed"
    exit 1
fi

print_success "Unitree third-party runtime setup completed!"
print_info "Location: ${TARGET_ARCH_DIR}"
