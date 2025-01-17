# Install script for directory: /home/lws/Firmware/src/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/airspeed/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/airspeed_validator/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/avoidance/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/battery/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/bezier/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/cdev/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/circuit_breaker/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/collision_prevention/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/controllib/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/conversion/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/drivers/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/ecl/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/flight_tasks/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/hysteresis/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/l1/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/landing_slope/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/led/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/mathlib/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/mixer/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/mixer_module/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/output_limit/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/perf/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/pid/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/rc/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/sensor_calibration/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/slew_rate/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/systemlib/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/tecs/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/terrain_estimation/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/tunes/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/version/cmake_install.cmake")
  include("/home/lws/Firmware/build/px4_fmu-v5_default/external/Build/px4io_firmware/src/lib/weather_vane/cmake_install.cmake")

endif()

