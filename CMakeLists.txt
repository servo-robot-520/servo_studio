cmake_minimum_required(VERSION 3.22)
project(servo_studio LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# 飞特驱动源码路径：-DFEETECH_DRIVER_DIR=<path> 指定；未配置时从上一级目录寻找。
set(FEETECH_DRIVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../feetech_servo_ros2_control"
        CACHE PATH "feetech_servo_ros2_control 源码路径")
if (NOT EXISTS "${FEETECH_DRIVER_DIR}/include/feetech_servo_ros2_control/scs_protocol.hpp")
    message(FATAL_ERROR
            "feetech_servo_ros2_control not found at '${FEETECH_DRIVER_DIR}'. "
            "Pass -DFEETECH_DRIVER_DIR=<path-to-feetech_servo_ros2_control>.")
endif ()
message(STATUS "feetech_servo_ros2_control: ${FEETECH_DRIVER_DIR}")

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
add_library(servo_backend_feetech STATIC
        src/backends/feetech/src/feetech_driver.cpp)
target_include_directories(servo_backend_feetech PUBLIC src/backends/feetech/include)
target_link_libraries(servo_backend_feetech PUBLIC servo_core feetech_scs_core)

# 无 GUI 冒烟工具。
add_executable(servo_smoke tools/servo_smoke.cpp)
target_link_libraries(servo_smoke PRIVATE servo_backend_feetech)

install(TARGETS servo_smoke RUNTIME DESTINATION bin)

include(CTest)
if (BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(test_feetech_driver test/test_feetech_driver.cpp)
    target_include_directories(test_feetech_driver PRIVATE test)
    target_link_libraries(test_feetech_driver PRIVATE servo_backend_feetech GTest::gtest_main)
    add_test(NAME test_feetech_driver COMMAND test_feetech_driver)

    add_executable(test_bus_session test/test_bus_session.cpp)
    target_link_libraries(test_bus_session PRIVATE servo_core GTest::gtest_main)
    add_test(NAME test_bus_session COMMAND test_bus_session)
endif ()
