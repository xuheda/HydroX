# GNU Arm Embedded toolchain used by the Pixhawk 6C FMU (STM32H743).
#
# This toolchain selects the CPU ABI only. The Pixhawk bootloader application
# offset, memory regions, startup configuration, and devices are supplied by
# the pinned NuttX board port through the `hydrox_pixhawk6c_bsp` bridge target.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(HYDROX_ARM_TOOLCHAIN_PREFIX "arm-none-eabi-" CACHE STRING
  "GNU Arm Embedded toolchain executable prefix")

set(CMAKE_C_COMPILER   "${HYDROX_ARM_TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${HYDROX_ARM_TOOLCHAIN_PREFIX}g++")
set(CMAKE_ASM_COMPILER "${HYDROX_ARM_TOOLCHAIN_PREFIX}gcc")
set(CMAKE_OBJCOPY      "${HYDROX_ARM_TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_SIZE         "${HYDROX_ARM_TOOLCHAIN_PREFIX}size")

set(HYDROX_PIXHAWK6C_CPU_FLAGS
  "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
set(HYDROX_PIXHAWK6C_COMMON_FLAGS
  "${HYDROX_PIXHAWK6C_CPU_FLAGS} -ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS_INIT
  "${HYDROX_PIXHAWK6C_COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT
  "${HYDROX_PIXHAWK6C_COMMON_FLAGS} -fno-exceptions -fno-rtti -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS_INIT
  "${HYDROX_PIXHAWK6C_CPU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT
  "${HYDROX_PIXHAWK6C_CPU_FLAGS} -specs=nano.specs -specs=nosys.specs")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
