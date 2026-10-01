# XLog writer

This directory owns the XLog writer, record layouts, and schema for both
HydroX and the ROS `hydrox_xlog` package. It depends only on C++17.

Use `add_subdirectory(libraries/xlog)` and link `hydrox_xlog`. Its public
include directory provides `xlog_writer.h`. The HydroX core links this target
publicly, so its consumers inherit the writer headers and library.

The ROS package builds this same target and installs its headers and library
through ament. There is no separate ROS source copy. When building that
package outside the OceanX checkout, set `HYDROX_SOURCE_DIR` to the HydroX
checkout root. Changes to record layouts and the schema belong here.

The existing HydroX XLog tests exercise record serialization, checksums,
segmentation, reading, and control replay. Recorded-file version handling is
part of the data format, independent of the removed source compatibility APIs.
