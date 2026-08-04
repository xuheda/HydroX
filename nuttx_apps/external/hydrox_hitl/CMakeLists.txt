if(CONFIG_HYDROX_HITL)
  if(NOT DEFINED HYDROX_SOURCE_DIR)
    message(FATAL_ERROR
      "HYDROX_SOURCE_DIR must point to the HydroX repository root")
  endif()

  get_filename_component(HYDROX_SOURCE_DIR
    "${HYDROX_SOURCE_DIR}" ABSOLUTE)
  set(HYDROX_FMUV6C_BOARD_DIR "${HYDROX_SOURCE_DIR}/boards/fmu_v6c")

  set(HYDROX_HITL_SOURCES
    "${HYDROX_SOURCE_DIR}/apps/pixhawk6c_hitl/main.cpp"
    "${HYDROX_FMUV6C_BOARD_DIR}/src/hydrox_hitl_board.cpp"
    "${HYDROX_SOURCE_DIR}/platform/nuttx/src/nuttx_clock.cpp"
    "${HYDROX_SOURCE_DIR}/platform/nuttx/src/nuttx_sleeper.cpp"
    "${HYDROX_SOURCE_DIR}/platform/nuttx/src/nuttx_serial_byte_stream.cpp"
    "${HYDROX_SOURCE_DIR}/platform/nuttx/src/nuttx_watchdog.cpp"
    "${HYDROX_SOURCE_DIR}/src/geodesy.cpp"
    "${HYDROX_SOURCE_DIR}/third_party/geographiclib-c/src/geodesic.cpp"
    "${HYDROX_SOURCE_DIR}/src/mavlink_hil.cpp"
    "${HYDROX_SOURCE_DIR}/src/runtime/hitl/mavlink_signing_disabled.cpp"
    "${HYDROX_SOURCE_DIR}/src/estimation_profile.cpp"
    "${HYDROX_SOURCE_DIR}/src/sensor_adapter.cpp"
    "${HYDROX_SOURCE_DIR}/src/ekf.cpp"
    "${HYDROX_SOURCE_DIR}/src/runtime/hil_runtime.cpp"
    "${HYDROX_SOURCE_DIR}/src/runtime/hitl_supervisor.cpp"
    "${HYDROX_SOURCE_DIR}/src/runtime/hitl_command_codec.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/ref_model.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/depth_pid.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/pitch_pid.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/heading_smc.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/inertia_gain_scaling.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/control_allocator.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/gnc_controller.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/thruster_allocator.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/thruster_controller.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/surface_allocator.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/surface_controller.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/multirotor_allocator.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/multirotor_controller.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/fixedwing_allocator.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/fixedwing_controller.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/vtol_allocator.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/vtol_controller.cpp"
    "${HYDROX_SOURCE_DIR}/src/gnc/control_factory.cpp"
  )

  foreach(HYDROX_SOURCE IN LISTS HYDROX_HITL_SOURCES)
    if(NOT EXISTS "${HYDROX_SOURCE}")
      message(FATAL_ERROR "Missing HydroX HITL source: ${HYDROX_SOURCE}")
    endif()
  endforeach()

  if(NOT EXISTS "${HYDROX_SOURCE_DIR}/third_party/eigen/Eigen/Core")
    message(FATAL_ERROR
      "Eigen 3.4 is required at ${HYDROX_SOURCE_DIR}/third_party/eigen")
  endif()

  nuttx_add_application(
    NAME hydrox_hitl
    PRIORITY ${CONFIG_HYDROX_HITL_PRIORITY}
    STACKSIZE ${CONFIG_HYDROX_HITL_STACKSIZE}
    NO_MAIN_ALIAS
    SRCS ${HYDROX_HITL_SOURCES}
    INCLUDE_DIRECTORIES
      "${HYDROX_SOURCE_DIR}/include"
      "${HYDROX_SOURCE_DIR}/platform/api/include"
      "${HYDROX_SOURCE_DIR}/platform/nuttx/include"
      "${HYDROX_SOURCE_DIR}/third_party/eigen"
      "${HYDROX_SOURCE_DIR}/third_party/geographiclib-c/src"
      "${HYDROX_FMUV6C_BOARD_DIR}/src"
    DEFINITIONS
      HYDROX_EMBEDDED=1
      HYDROX_HITL_ONLY=1
      HYDROX_TRANSPORT_UART=1
      STM32H7xx=1
      STM32H743xx=1
    COMPILE_FLAGS
      -ffunction-sections
      -fdata-sections
      -fno-exceptions
      -fno-rtti
      -fno-threadsafe-statics
  )

  set(HYDROX_FIRMWARE_DIR "${CMAKE_BINARY_DIR}/firmware")
  set(HYDROX_FIRMWARE_BASENAME "hydrox_pixhawk6cmini_hitl")
  add_custom_target(hydrox_pixhawk6cmini_hitl
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${HYDROX_FIRMWARE_DIR}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_BINARY_DIR}/nuttx"
      "${HYDROX_FIRMWARE_DIR}/${HYDROX_FIRMWARE_BASENAME}.elf"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_BINARY_DIR}/nuttx.bin"
      "${HYDROX_FIRMWARE_DIR}/${HYDROX_FIRMWARE_BASENAME}.bin"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_BINARY_DIR}/nuttx.hex"
      "${HYDROX_FIRMWARE_DIR}/${HYDROX_FIRMWARE_BASENAME}.hex"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_BINARY_DIR}/nuttx.map"
      "${HYDROX_FIRMWARE_DIR}/${HYDROX_FIRMWARE_BASENAME}.map"
    COMMAND "${Python3_EXECUTABLE}"
      "${HYDROX_SOURCE_DIR}/tools/package_px4.py"
      --prototype "${HYDROX_FMUV6C_BOARD_DIR}/firmware.prototype"
      --image "${CMAKE_BINARY_DIR}/nuttx.bin"
      --output "${HYDROX_FIRMWARE_DIR}/${HYDROX_FIRMWARE_BASENAME}.px4"
    DEPENDS nuttx_post
    COMMENT "Packaging HydroX Pixhawk 6C Mini / FMUv6C HITL firmware"
    VERBATIM
  )
endif()