cmake_minimum_required(VERSION 3.16)

set(HYDROX_TARGET "HOST" CACHE STRING "HydroX build target")
set_property(CACHE HYDROX_TARGET PROPERTY STRINGS HOST PIXHAWK6C_HITL)
if(HYDROX_TARGET STREQUAL "STM32")
  message(DEPRECATION
    "HYDROX_TARGET=STM32 is deprecated; use HYDROX_TARGET=PIXHAWK6C_HITL")
  set(HYDROX_TARGET "PIXHAWK6C_HITL" CACHE STRING "HydroX build target" FORCE)
endif()

if(HYDROX_TARGET STREQUAL "PIXHAWK6C_HITL")
  project(HydroX VERSION 0.1 LANGUAGES C CXX ASM)
else()
  project(HydroX VERSION 0.1 LANGUAGES CXX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(platform/api)
add_subdirectory(src/runtime)

# The Pixhawk 6C HITL firmware is a separate NuttX build. Keep host
# dependencies (Threads, OpenSSL, DDS workers, filesystem logging, and tests)
# out of its configuration entirely.
if(HYDROX_TARGET STREQUAL "PIXHAWK6C_HITL")
  include(cmake/targets/pixhawk6c_hitl.cmake)
  return()
endif()

find_package(Threads REQUIRED)
add_subdirectory(platform/host)

# ---------------------------------------------------------------------------
# Build Options
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
option(HYDROX_BUILD_TESTS "Build HydroX unit tests" ON)
option(HYDROX_ENABLE_RESIDUAL_RL "Enable optional learned wrench residual hook in hydrox_sitl" OFF)

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
# Third-party libraries
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

# Micro XRCE-DDS Client (Cross-platform: Windows / Linux / STM32)
# Automatically downloaded via FetchContent. If your network cannot reach
# GitHub, clone the repositories manually into third_party/ and adjust the
# FetchContent_Declare calls below, or set FETCHCONTENT_FULLY_DISCONNECTED.
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
# HydroX core library
# ---------------------------------------------------------------------------
set(HYDROX_CORE_SOURCES
  src/geodesy.cpp
  third_party/geographiclib-c/src/geodesic.cpp
  src/mavlink_hil.cpp
  src/mavlink_signing.cpp
  src/estimation_profile.cpp
  src/sensor_adapter.cpp
  src/ekf.cpp
  src/runtime/hil_runtime.cpp
  src/runtime/hitl_supervisor.cpp
  src/runtime/hitl_command_codec.cpp
  src/learning/residual_rl.cpp
  src/learning/control_shadow.cpp
  src/gnc/ref_model.cpp
  src/gnc/depth_pid.cpp
  src/gnc/pitch_pid.cpp
  src/gnc/heading_smc.cpp
  src/gnc/inertia_gain_scaling.cpp
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
  src/dds_publisher.cpp
  src/dds_setpoint_codec.cpp
  src/fossen_vehicle_params.cpp
  src/vehicle_bundle.cpp
  src/xlog_writer.cpp
)

add_library(hydrox STATIC ${HYDROX_CORE_SOURCES})

target_include_directories(hydrox PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_include_directories(hydrox PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/geographiclib-c/src
)

target_compile_definitions(hydrox PUBLIC HYDROX_DDS_ENABLED)

target_link_libraries(hydrox PUBLIC
  hydrox_runtime_api
  microxrcedds_client
  Threads::Threads
)

if(WIN32)
  target_link_libraries(hydrox PUBLIC ws2_32 bcrypt)
else()
  find_package(OpenSSL REQUIRED)
  target_link_libraries(hydrox PUBLIC OpenSSL::Crypto)
endif()

# ---------------------------------------------------------------------------
# SITL application (PC, TCP transport)
# ---------------------------------------------------------------------------
add_executable(hydrox_sitl
  apps/sitl/main.cpp
  src/sitl/sitl_config.cpp
  src/sitl/dds_worker.cpp
  src/sitl/sitl_platform.cpp
  src/sitl/sitl_xlog.cpp
  src/transport/tcp_transport.cpp
)

target_include_directories(hydrox_sitl PRIVATE src)
target_compile_definitions(hydrox_sitl PRIVATE HYDROX_TRANSPORT_TCP)
if(HYDROX_ENABLE_RESIDUAL_RL)
  target_compile_definitions(hydrox_sitl PRIVATE HYDROX_ENABLE_RESIDUAL_RL)
endif()
target_link_libraries(hydrox_sitl PRIVATE hydrox hydrox_platform_host)

# ---------------------------------------------------------------------------
# HITL router (transparent MAVLink2 TCP <-> serial bridge)
# ---------------------------------------------------------------------------
add_executable(hydrox_hitl_router
  apps/hitl_router/main.cpp
  src/transport/tcp_transport.cpp
)
target_link_libraries(hydrox_hitl_router PRIVATE
  hydrox_runtime_api
  hydrox_platform_host
)
if(WIN32)
  target_link_libraries(hydrox_hitl_router PRIVATE ws2_32)
endif()

# ---------------------------------------------------------------------------
# Offline learning shadow replay (no transport, DDS, UE, or actuator authority)
# ---------------------------------------------------------------------------
add_executable(hydrox_control_shadow apps/control_shadow/main.cpp)
target_link_libraries(hydrox_control_shadow PRIVATE hydrox)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(HYDROX_BUILD_TESTS)
  enable_testing()

  add_executable(test_dds_cdr tests/test_dds_cdr.cpp)
  target_link_libraries(test_dds_cdr PRIVATE hydrox)
  add_test(NAME test_dds_cdr COMMAND test_dds_cdr)

  add_executable(test_dds_cdr_writer tests/test_dds_cdr_writer.cpp)
  target_link_libraries(test_dds_cdr_writer PRIVATE hydrox)
  add_test(NAME test_dds_cdr_writer COMMAND test_dds_cdr_writer)

  add_executable(test_latest_value_mailbox tests/test_latest_value_mailbox.cpp)
  target_link_libraries(test_latest_value_mailbox PRIVATE hydrox)
  add_test(NAME test_latest_value_mailbox COMMAND test_latest_value_mailbox)

  add_executable(test_nonblocking_frame_sender tests/test_nonblocking_frame_sender.cpp)
  target_link_libraries(test_nonblocking_frame_sender PRIVATE hydrox)
  add_test(NAME test_nonblocking_frame_sender COMMAND test_nonblocking_frame_sender)

  add_executable(test_dds_connection_state tests/test_dds_connection_state.cpp)
  target_link_libraries(test_dds_connection_state PRIVATE hydrox)
  add_test(NAME test_dds_connection_state COMMAND test_dds_connection_state)

  add_executable(test_ue_control_session tests/test_ue_control_session.cpp)
  target_link_libraries(test_ue_control_session PRIVATE hydrox)
  add_test(NAME test_ue_control_session COMMAND test_ue_control_session)

  add_executable(test_hil_runtime tests/test_hil_runtime.cpp)
  target_link_libraries(test_hil_runtime PRIVATE hydrox)
  add_test(NAME test_hil_runtime COMMAND test_hil_runtime)

  add_executable(test_hitl_supervisor tests/test_hitl_supervisor.cpp)
  target_link_libraries(test_hitl_supervisor PRIVATE hydrox)
  add_test(NAME test_hitl_supervisor COMMAND test_hitl_supervisor)

  add_executable(test_hitl_command_codec tests/test_hitl_command_codec.cpp)
  target_link_libraries(test_hitl_command_codec PRIVATE hydrox)
  add_test(NAME test_hitl_command_codec COMMAND test_hitl_command_codec)
  add_executable(test_hitl_entry
    tests/test_hitl_entry.cpp
    apps/pixhawk6c_hitl/main.cpp
  )
  target_compile_definitions(test_hitl_entry PRIVATE
    HYDROX_EMBEDDED=1
    HYDROX_HITL_ONLY=1
  )
  target_link_libraries(test_hitl_entry PRIVATE hydrox)
  add_test(NAME test_hitl_entry COMMAND test_hitl_entry)

  add_executable(test_fossen_vehicle_params tests/test_fossen_vehicle_params.cpp)
  target_link_libraries(test_fossen_vehicle_params PRIVATE hydrox)
  add_test(NAME test_fossen_vehicle_params COMMAND test_fossen_vehicle_params)
  set_tests_properties(test_fossen_vehicle_params PROPERTIES
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
  set_tests_properties(test_fossen_vehicle_params PROPERTIES
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

  add_executable(test_vehicle_bundle tests/test_vehicle_bundle.cpp)
  target_link_libraries(test_vehicle_bundle PRIVATE hydrox)
  add_test(NAME test_vehicle_bundle COMMAND test_vehicle_bundle)
  set_tests_properties(test_vehicle_bundle PROPERTIES
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

  add_executable(test_air_allocators tests/test_air_allocators.cpp)
  target_link_libraries(test_air_allocators PRIVATE hydrox)
  add_test(NAME test_air_allocators COMMAND test_air_allocators)

  add_executable(test_surface_controller tests/test_surface_controller.cpp)
  target_link_libraries(test_surface_controller PRIVATE hydrox)
  add_test(NAME test_surface_controller COMMAND test_surface_controller)

  add_executable(test_thruster_allocator tests/test_thruster_allocator.cpp)
  target_link_libraries(test_thruster_allocator PRIVATE hydrox)
  add_test(NAME test_thruster_allocator COMMAND test_thruster_allocator)

  add_executable(test_fixedwing_controller tests/test_fixedwing_controller.cpp)
  target_link_libraries(test_fixedwing_controller PRIVATE hydrox)
  add_test(NAME test_fixedwing_controller COMMAND test_fixedwing_controller)

  add_executable(test_ekf tests/test_ekf.cpp)
  target_link_libraries(test_ekf PRIVATE hydrox)
  add_test(NAME test_ekf COMMAND test_ekf)

  add_executable(test_sitl_config
    tests/test_sitl_config.cpp
    src/sitl/sitl_config.cpp
  )
  target_include_directories(test_sitl_config PRIVATE src)
  target_link_libraries(test_sitl_config PRIVATE hydrox)
  add_test(NAME test_sitl_config COMMAND test_sitl_config)

  add_executable(test_control_feedback tests/test_control_feedback.cpp)
  target_include_directories(test_control_feedback PRIVATE src)
  target_link_libraries(test_control_feedback PRIVATE hydrox)
  add_test(NAME test_control_feedback COMMAND test_control_feedback)

  add_executable(test_sitl_xlog
    tests/test_sitl_xlog.cpp
    src/sitl/sitl_config.cpp
    src/sitl/sitl_xlog.cpp
  )
  target_include_directories(test_sitl_xlog PRIVATE src)
  target_link_libraries(test_sitl_xlog PRIVATE hydrox)
  add_test(NAME test_sitl_xlog COMMAND test_sitl_xlog)

  add_executable(test_sensor_adapter tests/test_sensor_adapter.cpp)
  target_link_libraries(test_sensor_adapter PRIVATE hydrox)
  add_test(NAME test_sensor_adapter COMMAND test_sensor_adapter)

  add_executable(test_geodesy tests/test_geodesy.cpp)
  target_link_libraries(test_geodesy PRIVATE hydrox)
  add_test(NAME test_geodesy COMMAND test_geodesy)

  add_executable(test_mavlink_signing tests/test_mavlink_signing.cpp)
  target_link_libraries(test_mavlink_signing PRIVATE hydrox)
  add_test(NAME test_mavlink_signing COMMAND test_mavlink_signing)

  add_executable(test_xlog_writer tests/test_xlog_writer.cpp)
  target_link_libraries(test_xlog_writer PRIVATE hydrox)
  add_test(NAME test_xlog_writer COMMAND test_xlog_writer)

  add_executable(test_residual_rl tests/test_residual_rl.cpp)
  target_link_libraries(test_residual_rl PRIVATE hydrox)
  add_test(NAME test_residual_rl COMMAND test_residual_rl)

  add_executable(test_control_shadow tests/test_control_shadow.cpp)
  target_link_libraries(test_control_shadow PRIVATE hydrox)
  add_test(NAME test_control_shadow COMMAND test_control_shadow)

  add_executable(test_platform_host tests/test_platform_host.cpp)
  target_link_libraries(test_platform_host PRIVATE
    hydrox_platform_host
    Threads::Threads
  )
  add_test(NAME test_platform_host COMMAND test_platform_host)

  add_executable(test_runtime_bus tests/test_runtime_bus.cpp)
  target_link_libraries(test_runtime_bus PRIVATE hydrox_runtime_api)
  add_test(NAME test_runtime_bus COMMAND test_runtime_bus)

  add_executable(test_periodic_scheduler tests/test_periodic_scheduler.cpp)
  target_link_libraries(test_periodic_scheduler PRIVATE hydrox_runtime_api)
  add_test(NAME test_periodic_scheduler COMMAND test_periodic_scheduler)
endif()
