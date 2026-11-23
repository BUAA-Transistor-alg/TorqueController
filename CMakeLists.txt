cmake_minimum_required(VERSION 3.10)

# ============================================================================
# TorqueController —— 单 yaw 云台力矩控制子模组
#
# 模块标识（pfx）= **tcs**（TorqueController Single），统一封装约定与同源的
# 双级 yaw 子模组（tcbs）保持一致。要点:
#   * target 名 / 产物文件名 / ctest 名一律带 tcs_ 前缀，库另提供 tcs::<原名>
#     形式的 ALIAS，供父工程 target_link_libraries 使用；
#   * 头文件物理位于 include/tcs/...，对内外一律通过 "tcs/..." 访问，因此
#     target_include_directories 只暴露 include 这一层（不再暴露
#     include/communication、include/mpc、include/c_api 等二级目录）；
#   * 所有 C++ 代码在 namespace tcs 内（内含 tcs::mcu / tcs::imu 子命名空间），
#     C ABI 的符号/类型/宏带 tcs_ / Tcs / TCS_ 前缀；
#   * 本文件既可**独立**构建（bash build.sh），也可被父工程以
#         add_subdirectory(<本目录> <binary_dir> EXCLUDE_FROM_ALL)
#     引入。作为子目录时**不**调用 project() / enable_testing()，也**不**使用
#     任何会污染父作用域的全局命令（CMAKE_* 全局设置 / add_compile_options /
#     include_directories），全部改用 target_* 形式；
#     为此 C++17 通过 target_compile_features(... cxx_std_17) 逐 target 指定，
#     find_library 结果等 cache 变量一律带 TCS_ 前缀，避免与父工程或兄弟
#     子模组互相覆盖。
# ============================================================================

set(TCS_IS_TOP_LEVEL OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(TCS_IS_TOP_LEVEL ON)
endif()

if(TCS_IS_TOP_LEVEL)
    project(TorqueController LANGUAGES CXX)
    # 独立构建时保持原有行为（未指定则默认 Release + -O2）；作为子目录时完全
    # 沿用父工程配置（以下 set 均为**目录作用域**变量，不影响父工程）
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE Release)
    endif()
    set(CMAKE_CXX_FLAGS_RELEASE "-O2")
    enable_testing()
endif()

# ============================================================================
# 依赖
# ============================================================================
find_library(TCS_UDEV_LIBRARY udev)
if(NOT TCS_UDEV_LIBRARY)
    message(FATAL_ERROR "libudev not found")
endif()
find_package(Ceres REQUIRED)

# 本仓库对外暴露的唯一 include 根目录（第二级目录名不再出现在搜索路径里）
set(TCS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

option(IS_INFANTRY_2 "启用 Infantry2 的 IMU 绕 Y 轴旋转 180 度安装补偿" OFF)

# ============================================================================
# 通信库（串口协议 + CRC + MCU 数据预处理 + yaw/底盘融合滤波器）
# ============================================================================
set(TCS_COMMUNICATION_SOURCES
    src/communication/CRC.cpp
    src/communication/ImuMountCompensation.cpp
    src/communication/McuDataPreprocessor.cpp
    src/communication/FusionFilter.cpp
)

add_library(tcs_communication STATIC ${TCS_COMMUNICATION_SOURCES})
add_library(tcs::communication ALIAS tcs_communication)
set_target_properties(tcs_communication PROPERTIES OUTPUT_NAME tcs_communication)
target_include_directories(tcs_communication PUBLIC ${TCS_INCLUDE_DIR})
target_compile_features(tcs_communication PUBLIC cxx_std_17)
target_link_libraries(tcs_communication PUBLIC ${TCS_UDEV_LIBRARY})

# ============================================================================
# MPC（求解器 / yaw MPC / MCU 控制封装）
# ============================================================================
set(TCS_MPC_SOURCES
    src/mpc/mpc_controller.cpp
    src/mpc/yaw_mpc_controller.cpp
    src/mpc/mcu_mpc_controller.cpp
    src/common/FrameRateCounter.cpp
)

# ============================================================================
# C API + 一体化封装动态库（c_api/ + src/RobotController.cpp；供 Python 与外部 cpp）
# ============================================================================
add_library(tcs_robot_comm_c SHARED
    src/c_api/RobotCommunicationC.cpp
    ${TCS_COMMUNICATION_SOURCES}
    ${TCS_MPC_SOURCES}
    src/RobotController.cpp
)
add_library(tcs::robot_comm_c ALIAS tcs_robot_comm_c)
set_target_properties(tcs_robot_comm_c PROPERTIES
    OUTPUT_NAME tcs_robot_comm_c
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_include_directories(tcs_robot_comm_c PUBLIC
    ${TCS_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)
target_compile_features(tcs_robot_comm_c PUBLIC cxx_std_17)
target_link_libraries(tcs_robot_comm_c PUBLIC ${TCS_UDEV_LIBRARY} ${CERES_LIBRARIES})

foreach(tcs_target tcs_communication tcs_robot_comm_c)
    target_compile_definitions(${tcs_target} PUBLIC
        IS_INFANTRY_2=$<BOOL:${IS_INFANTRY_2}>)
endforeach()

# ============================================================================
# 示例 / 工具
# ============================================================================
# 串口通信测试
add_executable(tcs_test_serial src/test_serial.cpp)
set_target_properties(tcs_test_serial PROPERTIES OUTPUT_NAME tcs_test_serial)
target_include_directories(tcs_test_serial PRIVATE ${TCS_INCLUDE_DIR})
target_compile_features(tcs_test_serial PRIVATE cxx_std_17)
target_link_libraries(tcs_test_serial PRIVATE tcs::communication)

# 独立构建默认包含姿态窗口；父工程可通过选项显式启用。
option(TCS_BUILD_POSE_VIEWER "构建 IMU/MCU 实时姿态显示程序" ${TCS_IS_TOP_LEVEL})
if(TCS_BUILD_POSE_VIEWER)
    find_package(SDL2 REQUIRED)
    set(OpenGL_GL_PREFERENCE GLVND)
    find_package(OpenGL REQUIRED)
    find_package(Freetype REQUIRED)
    find_package(Threads REQUIRED)
    add_executable(tcs_test_control src/test_control.cpp)
    set_target_properties(tcs_test_control PROPERTIES
        OUTPUT_NAME test_control
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    target_link_libraries(tcs_test_control PRIVATE tcs::communication SDL2::SDL2
        OpenGL::GL Freetype::Freetype Threads::Threads)
    target_compile_options(tcs_test_control PRIVATE -Wall -Wextra -Wpedantic)
endif()

# MCU 折叠状态显示
add_executable(tcs_fold_demo src/fold_demo.cpp)
set_target_properties(tcs_fold_demo PROPERTIES
    OUTPUT_NAME fold_demo
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(tcs_fold_demo PRIVATE tcs::communication)

# Pitch 轴标定程序
add_executable(tcs_pitch_calibration src/pitch_calibration.cpp)
set_target_properties(tcs_pitch_calibration PROPERTIES
    OUTPUT_NAME pitch_calibration
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_include_directories(tcs_pitch_calibration PRIVATE ${TCS_INCLUDE_DIR})
target_compile_features(tcs_pitch_calibration PRIVATE cxx_std_17)
target_link_libraries(tcs_pitch_calibration PRIVATE tcs::communication)

# 控制演示程序（使用 tcs::RobotController 一体化封装）
add_executable(tcs_control_demo src/control_demo.cpp)
set_target_properties(tcs_control_demo PROPERTIES
    OUTPUT_NAME control_demo
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_compile_features(tcs_control_demo PRIVATE cxx_std_17)
target_link_libraries(tcs_control_demo PRIVATE tcs::robot_comm_c)

# ============================================================================
# 测试（ctest 名带 tcs_ 前缀；父工程 enable_testing() 后不会与其它子模组撞名）
# ============================================================================
add_executable(tcs_test_c_api_smoke tests/test_c_api_smoke.cpp)
set_target_properties(tcs_test_c_api_smoke PROPERTIES OUTPUT_NAME tcs_test_c_api_smoke)
target_include_directories(tcs_test_c_api_smoke PRIVATE ${TCS_INCLUDE_DIR})
target_compile_features(tcs_test_c_api_smoke PRIVATE cxx_std_17)
target_link_libraries(tcs_test_c_api_smoke PRIVATE tcs::robot_comm_c)
add_test(NAME tcs_c_api_smoke COMMAND tcs_test_c_api_smoke)

find_package(Eigen3 REQUIRED)
add_executable(tcs_test_imu_mount tests/test_imu_mount.cpp)
target_link_libraries(tcs_test_imu_mount PRIVATE tcs::communication Eigen3::Eigen)
add_test(NAME tcs_imu_mount COMMAND tcs_test_imu_mount)
