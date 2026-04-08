#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/scripts/common.sh"

setup_inference_runtime() {
    print_header "[Setting up Inference Runtime]"
    local download_script="${SCRIPT_DIR}/scripts/download_inference_runtime.sh"

    if [ -f "$download_script" ]; then
        bash "$download_script" || {
            print_error "Failed to setup inference runtime"
            exit 1
        }
    fi
}

setup_mujoco() {
    print_header "[Setting up MuJoCo]"
    local download_script="${SCRIPT_DIR}/scripts/download_mujoco.sh"

    if [ -f "$download_script" ]; then
        bash "$download_script" || {
            print_error "Failed to setup MuJoCo"
            exit 1
        }
    fi
}

setup_robot_descriptions() {
    print_header "[Setting up Robot Descriptions]"
    local download_script="${SCRIPT_DIR}/scripts/download_robot_descriptions.sh"

    if [ -f "$download_script" ]; then
        bash "$download_script" || {
            print_error "Failed to setup robot descriptions"
            exit 1
        }
    fi
}

run_cmake_build() {
    local use_mujoco="$1"
    local build_target="$2"

    print_header "[Running CMake Build]"
    print_info "Building target: ${build_target}"
    print_separator

    if [ "$use_mujoco" = true ]; then
        cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON -DUSE_MUJOCO=ON
    else
        cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON
    fi

    cmake --build cmake_build -j"$(nproc 2>/dev/null || echo 4)" --target "${build_target}"
    print_success "Build completed!"
}

clean_workspace() {
    print_header "[Cleaning Workspace]"
    rm -rf build/ cmake_build/ devel/ install/ log/ logs/ .catkin_tools/
    print_success "Clean completed!"
}

show_usage() {
    print_header "[Build System Usage]"
    echo "Usage: $0 [OPTIONS]"
    echo
    echo "Options:"
    echo "  -c, --clean      Remove build artifacts"
    echo "  -m, --cmake      Build rl_real_go2 only"
    echo "  -mj, --mujoco    Build rl_sim_mujoco only"
    echo "  -h, --help       Show this help message"
    echo
    echo "Examples:"
    echo "  $0 -m     # build rl_real_go2"
    echo "  $0 -mj    # build rl_sim_mujoco"
    echo "  $0 -c"
}

main() {
    local clean_mode=false
    local cmake_mode=false
    local mujoco_mode=false

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c|--clean) clean_mode=true; shift ;;
            -m|--cmake) cmake_mode=true; shift ;;
            -mj|--mujoco) cmake_mode=true; mujoco_mode=true; shift ;;
            -h|--help) show_usage; exit 0 ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) print_error "Unexpected argument: $1"; show_usage; exit 1 ;;
        esac
    done

    if [ "$clean_mode" = true ]; then
        clean_workspace
        exit 0
    fi

    if [ "$cmake_mode" = false ]; then
        show_usage
        exit 1
    fi

    setup_inference_runtime

    if [ "$mujoco_mode" = true ]; then
        setup_robot_descriptions
        setup_mujoco
        run_cmake_build true rl_sim_mujoco
    else
        run_cmake_build false rl_real_go2
    fi
}

main "$@"
