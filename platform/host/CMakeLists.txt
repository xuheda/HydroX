add_library(hydrox_platform_host STATIC
  src/host_clock.cpp
  src/host_serial_byte_stream.cpp
  src/host_sleeper.cpp
)

target_include_directories(hydrox_platform_host PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hydrox_platform_host PUBLIC hydrox_platform_api)
