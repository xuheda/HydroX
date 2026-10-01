set(SRCS
  stm32_boot.c
  stm32_bringup.c
  stm32_usb.c
  fmu_v6c_safe_outputs.c
  fmu_v6c_reset.c
)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
