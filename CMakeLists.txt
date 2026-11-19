cmake_minimum_required(VERSION 3.22)
set(PROJECT_VERSION 0.0.1)

# 编译器选择：约定优先 g++-13；未安装时自动回退到更新的 g++（C++20 即可）。
# 可用 -DCMAKE_CXX_COMPILER 或 CXX= 显式覆盖。
if (NOT CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
    find_program(SERVO_STUDIO_GXX NAMES g++-13 g++-14 g++-15 g++-16 g++)
    if (SERVO_STUDIO_GXX)
        set(CMAKE_CXX_COMPILER "${SERVO_STUDIO_GXX}")
    endif ()
endif ()

project(servo_studio LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# 飞特驱动源码路径：-DFEETECH_DRIVER_DIR=<path> 指定；未配置时从上一级目录寻找。
set(FEETECH_DRIVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../feetech_scs_ros2_control"
        CACHE PATH "feetech_scs_ros2_control 源码路径")
if (NOT EXISTS "${FEETECH_DRIVER_DIR}/include/feetech_scs_ros2_control/scs_protocol.hpp")
    message(FATAL_ERROR
            "feetech_scs_ros2_control not found at '${FEETECH_DRIVER_DIR}'. "
            "Pass -DFEETECH_DRIVER_DIR=<path-to-feetech_scs_ros2_control>.")
endif ()
message(STATUS "feetech_scs_ros2_control: ${FEETECH_DRIVER_DIR}")

# 驱动核心源文件（无 ROS 依赖），编译为本工程的静态库。
add_library(feetech_scs_core STATIC
        "${FEETECH_DRIVER_DIR}/src/scs_serial_port.cpp"
        "${FEETECH_DRIVER_DIR}/src/scs_protocol.cpp")
target_include_directories(feetech_scs_core PUBLIC "${FEETECH_DRIVER_DIR}/include")

# 中间层：厂商无关接口、注册表与总线会话（采集线程）。
add_library(servo_core STATIC
        src/servo_core/src/registry.cpp
        src/servo_core/src/bus_session.cpp)
target_include_directories(servo_core PUBLIC src/servo_core/include)
find_package(Threads REQUIRED)
target_link_libraries(servo_core PUBLIC Threads::Threads)

# 飞特后端：把驱动核心适配到 IServoDriver。
add_library(servo_backend_feetech_scs STATIC
        src/backends/feetech_scs/src/feetech_scs_driver.cpp)
target_include_directories(servo_backend_feetech_scs PUBLIC src/backends/feetech_scs/include)
target_link_libraries(servo_backend_feetech_scs PUBLIC servo_core feetech_scs_core)

# 无 GUI 冒烟工具。
add_executable(servo_smoke tools/servo_smoke.cpp)
target_link_libraries(servo_smoke PRIVATE servo_backend_feetech_scs)

install(TARGETS servo_smoke RUNTIME DESTINATION bin)

# Qt6 GUI 应用。
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)

add_executable(servo_studio
        src/app/main.cpp
        src/app/main_window.cpp
        src/app/session_bridge.cpp
        src/app/params_backup.cpp
        src/app/panels/connection_panel.cpp
        src/app/panels/device_tree.cpp
        src/app/panels/status_table.cpp
        src/app/panels/chart_panel.cpp
        src/app/panels/inspector_panel.cpp
        src/app/panels/log_panel.cpp
        src/app/resources/app.qrc)
target_include_directories(servo_studio PRIVATE src)
target_link_libraries(servo_studio PRIVATE servo_backend_feetech_scs Qt6::Widgets Qt6::Charts)

install(TARGETS servo_studio RUNTIME DESTINATION bin)

include(CTest)
if (BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(test_feetech_scs_driver test/test_feetech_scs_driver.cpp)
    target_include_directories(test_feetech_scs_driver PRIVATE test)
    target_link_libraries(test_feetech_scs_driver PRIVATE servo_backend_feetech_scs GTest::gtest_main)
    add_test(NAME test_feetech_scs_driver COMMAND test_feetech_scs_driver)

    add_executable(test_bus_session test/test_bus_session.cpp)
    target_link_libraries(test_bus_session PRIVATE servo_core GTest::gtest_main)
    add_test(NAME test_bus_session COMMAND test_bus_session)

    add_executable(test_params_backup test/test_params_backup.cpp src/app/params_backup.cpp)
    target_include_directories(test_params_backup PRIVATE src)
    target_link_libraries(test_params_backup PRIVATE Qt6::Core GTest::gtest_main)
    add_test(NAME test_params_backup COMMAND test_params_backup)
endif ()
