# Third-party notices

HydroX uses the following third-party software. Their respective licenses govern
the use of their code; this file lists them for attribution and compliance.

## Eigen

- **Project**: Eigen
- **Website**: https://eigen.tuxfamily.org/
- **Repository**: https://gitlab.com/libeigen/eigen.git
- **License**: Mozilla Public License 2.0 (MPL-2.0)
- **Usage**: Header-only linear algebra library for vectors, matrices, and
  numerical computations in HydroX.

Eigen is not distributed as part of the HydroX source repository. Users must
clone it manually into `third_party/eigen` before building. After cloning, the
full MPL-2.0 license text can be found at `third_party/eigen/COPYING.MPL2`.

## Micro-CDR

- **Project**: Micro-CDR
- **Repository**: https://github.com/eProsima/Micro-CDR
- **License**: Apache License 2.0
- **Usage**: CDR serialization used by the Micro XRCE-DDS client.

Micro-CDR is downloaded automatically by CMake via `FetchContent` during the
first configure step.

## Micro-XRCE-DDS-Client

- **Project**: eProsima Micro XRCE-DDS Client
- **Repository**: https://github.com/eProsima/Micro-XRCE-DDS-Client
- **License**: Apache License 2.0
- **Usage**: DDS/XRCE client transport for ROS 2 and mission-system telemetry.

Micro-XRCE-DDS-Client is downloaded automatically by CMake via `FetchContent`
during the first configure step.

## MAVLink

- **Project**: MAVLink
- **Website**: https://mavlink.io/
- **Repository**: https://github.com/mavlink/mavlink
- **License**: GNU Lesser General Public License (LGPL) / MIT for generated
  header files (see upstream project for details)
- **Usage**: MAVLink HIL protocol for simulator-to-flight-control communication.

MAVLink headers are generated upstream and included in HydroX for protocol
serialization.

## License files

The full text of the Apache License 2.0 is included in this repository as
[`LICENSE`](LICENSE). The MPL-2.0 license text for Eigen is available in the
Eigen source tree after it is cloned into `third_party/eigen/`.

## Disclaimer

This file is provided for attribution purposes and does not constitute legal
advice. If you redistribute HydroX or its dependencies, please ensure compliance
with all applicable third-party licenses.
