# HydroX HITL-only firmware for the Holybro Pixhawk 6C (FMUv6C / STM32H743).
#
# This file defines the build contract and firmware artifacts. The final image
# is a NuttX firmware containing HydroX as an out-of-tree application. Board
# startup, the linker script, device registration, and the hardware watchdog
# belong to the in-tree `boards/pixhawk6c` target.

if(NOT CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR
    "PIXHAWK6C_HITL must be configured with "
    "-DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm_none_eabi_gcc.cmake")
endif()

set(HYDROX_PIXHAWK6C_BOARD_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/boards/pixhawk6c")
if(NOT EXISTS "${HYDROX_PIXHAWK6C_BOARD_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "The in-tree Pixhawk 6C BSP is not implemented yet. "
    "boards/pixhawk6c must provide the pinned NuttX board port, STM32H743 "
    "startup configuration, a bootloader-compatible linker script, device "
    "registration, watchdog support, and the CMake target "
    "hydrox_pixhawk6c_bsp. "
    "See boards/pixhawk6c/README.md. No .bin is generated until it exists.")
endif()

add_subdirectory(
  "${HYDROX_PIXHAWK6C_BOARD_DIR}"
  "${CMAKE_BINARY_DIR}/pixhawk6c_bsp"
)
if(NOT TARGET hydrox_pixhawk6c_bsp)
  message(FATAL_ERROR
    "The BSP must define a CMake target named hydrox_pixhawk6c_bsp")
endif()

set(HYDROX_PIXHAWK6C_HITL_CORE_SOURCES
  src/geodesy.cpp
  third_party/geographiclib-c/src/geodesic.cpp
  src/mavlink_hil.cpp
  src/runtime/hitl/mavlink_signing_disabled.cpp
  src/estimation_profile.cpp
  src/sensor_adapter.cpp
  src/ekf.cpp
  src/runtime/hil_runtime.cpp
  src/runtime/hitl_supervisor.cpp
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
)

add_library(hydrox_pixhawk6c_hitl_core STATIC
  ${HYDROX_PIXHAWK6C_HITL_CORE_SOURCES}
)
target_include_directories(hydrox_pixhawk6c_hitl_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen"
)
target_include_directories(hydrox_pixhawk6c_hitl_core PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/geographiclib-c/src"
)
target_compile_definitions(hydrox_pixhawk6c_hitl_core PUBLIC
  HYDROX_EMBEDDED=1
  HYDROX_HITL_ONLY=1
)
target_link_libraries(hydrox_pixhawk6c_hitl_core PUBLIC
  hydrox_runtime_api
)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen/Eigen/Core")
  message(FATAL_ERROR
    "Eigen 3.4 is required at HydroX/third_party/eigen for the HITL firmware")
endif()

add_executable(hydrox_pixhawk6c_hitl
  apps/pixhawk6c_hitl/main.cpp
)
target_include_directories(hydrox_pixhawk6c_hitl PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_definitions(hydrox_pixhawk6c_hitl PRIVATE
  HYDROX_TRANSPORT_UART=1
  STM32H7xx=1
  STM32H743xx=1
)
target_link_libraries(hydrox_pixhawk6c_hitl PRIVATE
  hydrox_pixhawk6c_hitl_core
  hydrox_pixhawk6c_bsp
)

set(HYDROX_PIXHAWK6C_FIRMWARE_DIR "${CMAKE_BINARY_DIR}/firmware")
file(MAKE_DIRECTORY "${HYDROX_PIXHAWK6C_FIRMWARE_DIR}")
set_target_properties(hydrox_pixhawk6c_hitl PROPERTIES
  OUTPUT_NAME "hydrox_pixhawk6c_hitl"
  SUFFIX ".elf"
  RUNTIME_OUTPUT_DIRECTORY "${HYDROX_PIXHAWK6C_FIRMWARE_DIR}"
)
target_link_options(hydrox_pixhawk6c_hitl PRIVATE
  "-Wl,--gc-sections"
  "-Wl,-Map=${HYDROX_PIXHAWK6C_FIRMWARE_DIR}/hydrox_pixhawk6c_hitl.map"
)

if(NOT CMAKE_OBJCOPY)
  message(FATAL_ERROR
    "CMAKE_OBJCOPY is not configured; use "
    "cmake/toolchains/arm_none_eabi_gcc.cmake")
endif()

add_custom_command(TARGET hydrox_pixhawk6c_hitl POST_BUILD
  COMMAND "${CMAKE_OBJCOPY}" -O binary
    "$<TARGET_FILE:hydrox_pixhawk6c_hitl>"
    "${HYDROX_PIXHAWK6C_FIRMWARE_DIR}/hydrox_pixhawk6c_hitl.bin"
  COMMENT "Generating hydrox_pixhawk6c_hitl.bin"
  VERBATIM
)

if(CMAKE_SIZE)
  add_custom_command(TARGET hydrox_pixhawk6c_hitl POST_BUILD
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:hydrox_pixhawk6c_hitl>"
    COMMENT "HydroX Pixhawk 6C HITL firmware size"
    VERBATIM
  )
endif()

message(STATUS
  "HydroX Pixhawk 6C HITL artifacts: ${HYDROX_PIXHAWK6C_FIRMWARE_DIR}")
