add_library(hydrox_runtime_api INTERFACE)

target_include_directories(hydrox_runtime_api INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hydrox_runtime_api INTERFACE hydrox_platform_api)
target_compile_features(hydrox_runtime_api INTERFACE cxx_std_17)
