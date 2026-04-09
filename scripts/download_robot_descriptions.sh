#!/bin/bash

# Validate bundled robot descriptions.
# Usage: ./download_robot_descriptions.sh [target_dir]
#   target_dir: Target directory (default: src/rl_sar_zoo)

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

source "${SCRIPT_DIR}/common.sh"

if [ $# -eq 0 ]; then
    TARGET_DIR="src/rl_sar_zoo"
else
    TARGET_DIR="$1"
fi

ROBOT_DESC_DIR="${PROJECT_ROOT}/${TARGET_DIR}"
ROBOT_DESC_VERSION_FILE="${ROBOT_DESC_DIR}/VERSION"
EXPECTED_VERSION="1.0.1"

get_current_version() {
    if [ -f "$ROBOT_DESC_VERSION_FILE" ]; then
        tr -d '[:space:]' < "$ROBOT_DESC_VERSION_FILE"
    else
        echo "unknown"
    fi
}

is_robot_descriptions_valid() {
    [ -d "${ROBOT_DESC_DIR}/go2_description" ] && [ -d "${ROBOT_DESC_DIR}/go2w_description" ]
}

print_header "[Robot Descriptions Setup]"

if ! is_robot_descriptions_valid; then
    print_error "Bundled robot descriptions are missing or incomplete"
    print_info "Expected path: ${ROBOT_DESC_DIR}"
    print_info "This repository now requires src/rl_sar_zoo to be present locally instead of cloning it during build."
    exit 1
fi

current_version="$(get_current_version)"
if [ "$current_version" != "$EXPECTED_VERSION" ]; then
    print_warning "Robot description version mismatch: found=${current_version}, expected=${EXPECTED_VERSION}"
fi

if [ -d "${ROBOT_DESC_DIR}/.git" ]; then
    print_warning "Embedded git metadata detected at ${ROBOT_DESC_DIR}/.git"
    print_info "Remove that directory before committing if you want rl_sar_zoo vendored directly in this repository."
fi

print_success "Bundled robot descriptions are available"
print_info "Installation path: ${ROBOT_DESC_DIR}"
print_info "Available descriptions: go2_description, go2w_description"
print_info "Version: ${current_version}"
