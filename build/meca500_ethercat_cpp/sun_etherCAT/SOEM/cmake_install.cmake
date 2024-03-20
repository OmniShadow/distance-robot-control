# Install script for directory: C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/regolatore")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "G:/msys2/ucrt64/bin/objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/distance-robot-control/build/meca500_ethercat_cpp/sun_etherCAT/SOEM/libsoem.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("C:/distance-robot-control/build/meca500_ethercat_cpp/sun_etherCAT/SOEM/CMakeFiles/soem.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake"
         "C:/distance-robot-control/build/meca500_ethercat_cpp/sun_etherCAT/SOEM/CMakeFiles/Export/39806c66e6e7fd9076eb39407f12ee6f/soemConfig.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/soem/cmake" TYPE FILE FILES "C:/distance-robot-control/build/meca500_ethercat_cpp/sun_etherCAT/SOEM/CMakeFiles/Export/39806c66e6e7fd9076eb39407f12ee6f/soemConfig.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/soem/cmake" TYPE FILE FILES "C:/distance-robot-control/build/meca500_ethercat_cpp/sun_etherCAT/SOEM/CMakeFiles/Export/39806c66e6e7fd9076eb39407f12ee6f/soemConfig-debug.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/soem" TYPE FILE FILES
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercat.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatbase.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatcoe.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatconfig.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatconfiglist.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatdc.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercateoe.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatfoe.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatmain.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatprint.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercatsoe.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/soem/ethercattype.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/osal/osal.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/osal/win32/inttypes.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/osal/win32/osal_defs.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/osal/win32/osal_win32.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/osal/win32/stdint.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/oshw/win32/nicdrv.h"
    "C:/distance-robot-control/meca500_ethercat_cpp/sun_etherCAT/SOEM/oshw/win32/oshw.h"
    )
endif()

