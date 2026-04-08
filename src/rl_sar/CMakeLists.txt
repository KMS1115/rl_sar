cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})
set(USE_CMAKE ON CACHE BOOL "Build with standalone CMake")
set(USE_MUJOCO OFF CACHE BOOL "Build with MuJoCo simulator support")
add_compile_definitions(USE_CMAKE)

get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

set(INFERENCE_RUNTIME_DIR "${PROJECT_ROOT_DIR}/library/inference_runtime")
set(MUJOCO_ROOT_DIR "${PROJECT_ROOT_DIR}/library/mujoco")

set(CMAKE_INSTALL_RPATH "")

# ONNX Runtime
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime-aarch64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
        set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime-x64")
    else()
        set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|ARM64")
        set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime-macos-arm64")
    else()
        set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime-macos-x64")
    endif()
else()
    set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")
endif()

if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)

    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
    endif()

    include_directories("${ONNX_RUNTIME_DIR}/include")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(LIB_EXT "dylib")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_EXT "lib")
    endif()

    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_NAMES onnxruntime.lib)
    else()
        set(LIB_NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT})
    endif()

    unset(ONNX_RUNTIME_LIB_PATH CACHE)
    find_library(ONNX_RUNTIME_LIB_PATH
        NAMES ${LIB_NAMES}
        PATHS "${ONNX_RUNTIME_DIR}/lib"
        NO_DEFAULT_PATH
    )

    if(ONNX_RUNTIME_LIB_PATH)
        if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            add_library(onnxruntime_prebuilt STATIC IMPORTED GLOBAL)
        else()
            add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
        endif()

        set_target_properties(onnxruntime_prebuilt PROPERTIES
            IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
        )

        set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
        list(APPEND CMAKE_INSTALL_RPATH "${ONNX_RUNTIME_DIR}/lib")
        message(STATUS "Found ONNX Runtime in ${ONNX_RUNTIME_DIR}: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        message(WARNING "ONNX Runtime library not found")
        set(USE_ONNX "OFF")
    endif()
else()
    message(WARNING "ONNX Runtime not found. Please run: bash scripts/download_inference_runtime.sh")
endif()

# LibTorch is intentionally disabled in this ONNX-only build.
set(USE_TORCH "OFF")
set(TORCH_LIBRARIES "")

if(USE_ONNX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

if(Python3_LIBRARIES)
    get_filename_component(PYTHON_LIB_DIR "${Python3_LIBRARIES}" DIRECTORY)
    list(APPEND CMAKE_INSTALL_RPATH "${PYTHON_LIB_DIR}")
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
    link_directories(/usr/local/lib)
    include_directories(${YAML_CPP_INCLUDE_DIR})
elseif(SYSTEM_TYPE STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
    link_directories(/opt/homebrew/lib)
    if(YAML_CPP_INCLUDE_DIR)
        include_directories(${YAML_CPP_INCLUDE_DIR})
    endif()

    find_library(OMP_LIB NAMES omp PATHS /opt/homebrew/lib /opt/homebrew/opt/libomp/lib)
    if(OMP_LIB)
        set(OPENMP_LIBRARIES ${OMP_LIB})
        get_filename_component(OMP_LIB_DIR "${OPENMP_LIBRARIES}" DIRECTORY)
        list(APPEND CMAKE_INSTALL_RPATH "${OMP_LIB_DIR}")
    else()
        message(WARNING "OpenMP not found. Please install: brew install libomp")
    endif()
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(ARCH_DIR "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(ARCH_DIR "x86_64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples build for unitree_sdk2" FORCE)
    add_subdirectory(library/thirdparty/robot_sdk/unitree/unitree_sdk2)
endif()

include_directories(
    include
    library/core/fsm
    library/core/inference_runtime
    library/core/logger
    library/core/loop
    library/core/matplotlibcpp
    library/core/motion_loader
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/vector_math
    fsm_robot
)

add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
endif()

add_library(motion_loader library/core/motion_loader/motion_loader.cpp)

add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC
    motion_loader
    inference_runtime
    Python3::Python
    Python3::Module
    TBB::tbb
)
find_package(Python3 COMPONENTS NumPy QUIET)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()

add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)

if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_go2 src/rl_real_go2.cpp)
    target_link_libraries(rl_real_go2
        unitree_sdk2
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    set_target_properties(rl_real_go2 PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2/thirdparty/lib/${ARCH_DIR}"
    )
endif()

if(USE_MUJOCO)
    add_compile_definitions(USE_MUJOCO)

    if(SYSTEM_TYPE STREQUAL "Darwin")
        if(NOT EXISTS "${MUJOCO_ROOT_DIR}/Headers/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()

        set(MUJOCO_COMPAT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/mujoco_include")
        file(MAKE_DIRECTORY "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")
        file(GLOB MUJOCO_HEADERS "${MUJOCO_ROOT_DIR}/Headers/*.h")
        file(COPY ${MUJOCO_HEADERS} DESTINATION "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")

        set(MUJOCO_INCLUDE_DIR "${MUJOCO_COMPAT_INCLUDE_DIR}")
        set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/Versions/Current")
        set(MUJOCO_LIB_NAME "libmujoco.3.2.7.dylib")
    else()
        if(NOT EXISTS "${MUJOCO_ROOT_DIR}/include/mujoco/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()
        set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
        set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/lib")

        if(SYSTEM_TYPE STREQUAL "Linux")
            set(MUJOCO_LIB_NAME "libmujoco.so.3.2.7")
        elseif(SYSTEM_TYPE STREQUAL "Windows")
            set(MUJOCO_LIB_NAME "mujoco.dll")
        else()
            message(FATAL_ERROR "Unsupported platform: ${SYSTEM_TYPE}")
        endif()
    endif()

    list(APPEND CMAKE_INSTALL_RPATH "${MUJOCO_LIB_DIR}")
    find_package(glfw3 REQUIRED)

    add_library(mujoco_prebuilt SHARED IMPORTED)
    set_target_properties(mujoco_prebuilt PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_LIB_DIR}/${MUJOCO_LIB_NAME}"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
        IMPORTED_NO_SONAME TRUE
    )

    include_directories(
        ${MUJOCO_INCLUDE_DIR}
        library/thirdparty/joystick
    )

    file(GLOB MUJOCO_SIMULATE_SRC library/thirdparty/mujoco_simulate/*.cc)
    set(JOYSTICK_SRC library/thirdparty/joystick/joystick.cc)

    if(SYSTEM_TYPE STREQUAL "Darwin")
        file(GLOB MUJOCO_SIMULATE_MM library/thirdparty/mujoco_simulate/*.mm)
        list(APPEND MUJOCO_SIMULATE_SRC ${MUJOCO_SIMULATE_MM})
    endif()

    add_executable(rl_sim_mujoco src/rl_sim_mujoco.cpp ${MUJOCO_SIMULATE_SRC} ${JOYSTICK_SRC})

    if(SYSTEM_TYPE STREQUAL "Darwin")
        set_source_files_properties(
            ${MUJOCO_SIMULATE_MM}
            PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations"
        )
    endif()

    target_include_directories(rl_sim_mujoco PRIVATE
        library/thirdparty/joystick
        library/thirdparty/mujoco_simulate
    )

    target_link_libraries(rl_sim_mujoco
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
        mujoco_prebuilt
        glfw
    )

    if(SYSTEM_TYPE STREQUAL "Darwin")
        if(OPENMP_LIBRARIES)
            target_link_libraries(rl_sim_mujoco ${OPENMP_LIBRARIES})
        endif()

        target_link_libraries(rl_sim_mujoco
            "-framework CoreVideo"
            "-framework OpenGL"
            "-framework AppKit"
        )
    endif()
endif()

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")
message(STATUS "USE_MUJOCO: ${USE_MUJOCO}")
message(STATUS "USE_ONNX: ${USE_ONNX}")
message(STATUS "========================================")
