cmake_minimum_required(VERSION 3.16)
project(hydrox_xlog_core VERSION 1.0.0 LANGUAGES CXX)

# Shared by HydroX and the ROS package; no autopilot or ROS dependencies.
add_library(hydrox_xlog STATIC src/xlog_writer.cpp)
target_compile_features(hydrox_xlog PUBLIC cxx_std_17)
set_target_properties(hydrox_xlog PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hydrox_xlog PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
