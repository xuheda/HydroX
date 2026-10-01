add_library(hydrox_platform_api INTERFACE)

target_include_directories(hydrox_platform_api INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(hydrox_platform_api INTERFACE cxx_std_17)
