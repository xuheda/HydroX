set(SRCS
  stm32_boot.c
  stm32_bringup.c
  fmu_v6c_safe_outputs.c
  fmu_v6c_reset.c
)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
endif()

target_sources(board PRIVATE ${SRCS})