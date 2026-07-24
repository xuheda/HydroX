cmake_minimum_required(VERSION 3.16)
project(HydroX VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Build Options
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
option(HYDROX_BUILD_TESTS "Build HydroX unit tests" ON)

if(MSVC)
  add_compile_options(/W3 /WX- /EHsc /utf-8)
  add_compile_options($<$<CONFIG:Release>:/O2>)
  add_compile_options($<$<CONFIG:Debug>:/Od>)
  add_compile_options($<$<CONFIG:Debug>:/Zi>)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
  add_compile_options(
    -Wall -Wextra
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Debug>:-g -O0>
  )
endif()

# ---------------------------------------------------------------------------
# Third-party libraries (header-only)
# ---------------------------------------------------------------------------
# Eigen: Clone to third_party/eigen before building.
#   git clone --depth 1 --branch 3.4.0 \
#       https://gitlab.com/libeigen/eigen.git third_party/eigen
# For users in China, a gitee mirror can be used:
#   git clone --depth 1 --branch 3.4.0 \
#       https://gitee.com/mirrors/eigen.git third_party/eigen
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen")
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
  message(FATAL_ERROR
    "Eigen not found. Please run one of the following commands\n"
    "from the project root before building:\n"
    "  git clone --depth 1 --branch 3.4.0 "
    "https://gitlab.com/libeigen/eigen.git third_party/eigen\n"
    "or, for users in China:\n"
    "  git clone --depth 1 --branch 3.4.0 "
    "https://gitee.com/mirrors/eigen.git third_party/eigen")
endif()

# ---------------------------------------------------------------------------
# Micro XRCE-DDS Client (Cross-platform: Windows / Linux / STM32)
# Automatically downloaded via FetchContent. If your network cannot reach
# GitHub, clone the repositories manually into third_party/ and adjust the
# FetchContent_Declare calls below, or set FETCHCONTENT_FULLY_DISCONNECTED.
# ---------------------------------------------------------------------------
include(FetchContent)
set(UCDR_SUPERBUILD        OFF CACHE BOOL "" FORCE)
set(UCDR_BUILD_TESTS       OFF CACHE BOOL "" FORCE)
set(UCDR_BUILD_EXAMPLES    OFF CACHE BOOL "" FORCE)
set(UCLIENT_SUPERBUILD     OFF CACHE BOOL "" FORCE)
set(UXRCE_BUILD_TESTS       OFF CACHE BOOL "" FORCE)
set(UXRCE_BUILD_EXAMPLES    OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_UDP     ON  CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_TCP     OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_SERIAL  OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_CUSTOM_TRANSPORT OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_DISCOVERY OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  microcdr
  GIT_REPOSITORY https://github.com/eProsima/Micro-CDR.git
  GIT_TAG        v2.0.1
)
FetchContent_Declare(
  uxr_client
  GIT_REPOSITORY https://github.com/eProsima/Micro-XRCE-DDS-Client.git
  GIT_TAG        v2.4.2
)
FetchContent_MakeAvailable(microcdr)
set(_microcdr_redirect_dir "${CMAKE_BINARY_DIR}/microcdr_package")
file(MAKE_DIRECTORY "${_microcdr_redirect_dir}")
file(WRITE "${_microcdr_redirect_dir}/microcdr-config.cmake"
  "if(NOT TARGET microcdr)\n"
  "  message(FATAL_ERROR \"microcdr target is not available\")\n"
  "endif()\n"
  "set(microcdr_FOUND TRUE)\n"
)
file(WRITE "${_microcdr_redirect_dir}/microcdr-config-version.cmake"
  "set(PACKAGE_VERSION \"2.0.1\")\n"
  "if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)\n"
  "  set(PACKAGE_VERSION_EXACT TRUE)\n"
  "  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
  "elseif(PACKAGE_FIND_VERSION VERSION_LESS PACKAGE_VERSION)\n"
  "  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
  "endif()\n"
)
set(microcdr_DIR "${_microcdr_redirect_dir}" CACHE PATH "" FORCE)
FetchContent_MakeAvailable(uxr_client)

# ---------------------------------------------------------------------------
# Source Files
# ---------------------------------------------------------------------------
set(FC_SOURCES
  src/mavlink_hil.cpp
  src/mavlink_signing.cpp
  src/sensor_adapter.cpp
  src/ekf.cpp
  src/gnc/ref_model.cpp
  src/gnc/depth_pid.cpp
  src/gnc/pitch_pid.cpp
  src/gnc/heading_smc.cpp
  src/gnc/control_allocator.cpp
  src/gnc/gnc_controller.cpp
  src/gnc/thruster_allocator.cpp
  src/gnc/thruster_controller.cpp
  src/gnc/surface_allocator.cpp
  src/gnc/surface_controller.cpp
  src/gnc/multirotor_allocator.cpp
  src/gnc/multirotor_controller.cpp
  src/gnc/fixedwing_allocator.cpp
  src/gnc/fixedwing_controller.cpp
  src/gnc/vtol_allocator.cpp
  src/gnc/vtol_controller.cpp
  src/gnc/control_factory.cpp
)

# XLog is a reusable library shared with the ROS diagnostics package.
set(HYDROX_XLOG_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../ros/src/hydrox_xlog")

# ---------------------------------------------------------------------------
# SITL Executable (Runs on PC, TCP transport)
# ---------------------------------------------------------------------------
# Analogous to px4_sitl
add_executable(hydrox_sitl
  ${FC_SOURCES}
  src/dds_publisher.cpp
  src/dds_setpoint_codec.cpp
  src/fossen_vehicle_params.cpp
  ${HYDROX_XLOG_DIR}/src/xlog_writer.cpp
  src/sitl/sitl_config.cpp
  src/sitl/dds_worker.cpp
  src/sitl/sitl_platform.cpp
  src/sitl/sitl_xlog.cpp
  src/transport/tcp_transport.cpp
  src/main_sitl.cpp
)

target_include_directories(hydrox_sitl PRIVATE
  include
  ${HYDROX_XLOG_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

# SITL uses TCP transport + DDS publisher
target_compile_definitions(hydrox_sitl PRIVATE HYDROX_TRANSPORT_TCP HYDROX_DDS_ENABLED)

target_link_libraries(hydrox_sitl PRIVATE microxrcedds_client Threads::Threads)

if(WIN32)
  target_link_libraries(hydrox_sitl PRIVATE ws2_32 bcrypt)
else()
  find_package(OpenSSL REQUIRED)
  target_link_libraries(hydrox_sitl PRIVATE OpenSSL::Crypto)
endif()

if(HYDROX_BUILD_TESTS)
  enable_testing()
  add_executable(test_dds_cdr
    tests/test_dds_cdr.cpp
    src/dds_setpoint_codec.cpp
  )
  target_include_directories(test_dds_cdr PRIVATE
    include
  )
  target_link_libraries(test_dds_cdr PRIVATE microcdr)
  add_test(NAME test_dds_cdr COMMAND test_dds_cdr)

  add_executable(test_dds_cdr_writer
    tests/test_dds_cdr_writer.cpp
  )
  target_include_directories(test_dds_cdr_writer PRIVATE include)
  target_link_libraries(test_dds_cdr_writer PRIVATE microcdr)
  add_test(NAME test_dds_cdr_writer COMMAND test_dds_cdr_writer)

  add_executable(test_latest_value_mailbox
    tests/test_latest_value_mailbox.cpp
  )
  target_include_directories(test_latest_value_mailbox PRIVATE include)
  add_test(NAME test_latest_value_mailbox COMMAND test_latest_value_mailbox)

  add_executable(test_nonblocking_frame_sender
    tests/test_nonblocking_frame_sender.cpp
  )
  target_include_directories(test_nonblocking_frame_sender PRIVATE include)
  add_test(NAME test_nonblocking_frame_sender COMMAND test_nonblocking_frame_sender)

  add_executable(test_dds_connection_state
    tests/test_dds_connection_state.cpp
  )
  target_include_directories(test_dds_connection_state PRIVATE include)
  add_test(NAME test_dds_connection_state COMMAND test_dds_connection_state)

  add_executable(test_ue_control_session
    tests/test_ue_control_session.cpp
  )
  target_include_directories(test_ue_control_session PRIVATE include)
  add_test(NAME test_ue_control_session COMMAND test_ue_control_session)

  add_executable(test_fossen_vehicle_params
    tests/test_fossen_vehicle_params.cpp
    src/fossen_vehicle_params.cpp
  )
  target_include_directories(test_fossen_vehicle_params PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
  )
  add_test(NAME test_fossen_vehicle_params COMMAND test_fossen_vehicle_params)
  set_tests_properties(test_fossen_vehicle_params PROPERTIES
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/..")

  add_executable(test_air_allocators
    tests/test_air_allocators.cpp
    src/gnc/multirotor_allocator.cpp
    src/gnc/vtol_allocator.cpp
  )
  target_include_directories(test_air_allocators PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
  )
  add_test(NAME test_air_allocators COMMAND test_air_allocators)

  add_executable(test_ekf
    tests/test_ekf.cpp
    src/ekf.cpp
  )
  target_include_directories(test_ekf PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
  )
  add_test(NAME test_ekf COMMAND test_ekf)

  add_executable(test_sitl_config
    tests/test_sitl_config.cpp
    src/sitl/sitl_config.cpp
  )
  target_include_directories(test_sitl_config PRIVATE
    include
    src
    ${EIGEN3_INCLUDE_DIR}
  )
  add_test(NAME test_sitl_config COMMAND test_sitl_config)

  add_executable(test_sitl_xlog
    tests/test_sitl_xlog.cpp
    src/sitl/sitl_xlog.cpp
    src/sitl/sitl_config.cpp
    ${HYDROX_XLOG_DIR}/src/xlog_writer.cpp
    src/ekf.cpp
    src/fossen_vehicle_params.cpp
  )
  target_include_directories(test_sitl_xlog PRIVATE
    include
    src
    ${HYDROX_XLOG_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
  )
  add_test(NAME test_sitl_xlog COMMAND test_sitl_xlog)

  add_executable(test_sensor_adapter
    tests/test_sensor_adapter.cpp
    src/sensor_adapter.cpp
    src/mavlink_hil.cpp
    src/mavlink_signing.cpp
  )
  target_include_directories(test_sensor_adapter PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
  )
  add_test(NAME test_sensor_adapter COMMAND test_sensor_adapter)

  add_executable(test_mavlink_signing
    tests/test_mavlink_signing.cpp
    src/mavlink_hil.cpp
    src/mavlink_signing.cpp
  )
  target_include_directories(test_mavlink_signing PRIVATE include)
  if(WIN32)
    target_link_libraries(test_sensor_adapter PRIVATE bcrypt)
    target_link_libraries(test_mavlink_signing PRIVATE bcrypt)
  else()
    target_link_libraries(test_sensor_adapter PRIVATE OpenSSL::Crypto)
    target_link_libraries(test_mavlink_signing PRIVATE OpenSSL::Crypto)
  endif()
  add_test(NAME test_mavlink_signing COMMAND test_mavlink_signing)

  add_executable(test_xlog_writer
    tests/test_xlog_writer.cpp
    ${HYDROX_XLOG_DIR}/src/xlog_writer.cpp
  )
  target_include_directories(test_xlog_writer PRIVATE
    include
    ${HYDROX_XLOG_DIR}/include
  )
  add_test(NAME test_xlog_writer COMMAND test_xlog_writer)
endif()

# ---------------------------------------------------------------------------
# STM32 HIL Target (Cross-compilation, requires toolchain specification)
# cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_stm32h7.cmake -DHYDROX_TARGET=STM32 ..
# ---------------------------------------------------------------------------
if(HYDROX_TARGET STREQUAL "STM32")
  # Analogous to px4_fmu-v6c_default
  add_executable(hydrox_pixhawk6c
    ${FC_SOURCES}
    src/transport/uart_transport.cpp
    src/main_stm32.cpp
  )
  target_include_directories(hydrox_pixhawk6c PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
  )
  target_compile_definitions(hydrox_pixhawk6c PRIVATE HYDROX_TRANSPORT_UART)
  # FreeRTOS and STM32 HAL are provided by the toolchain file
endif()
