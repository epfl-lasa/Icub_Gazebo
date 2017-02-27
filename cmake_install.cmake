# Install script for directory: /home/neda/iCube/gazebo-yarp-plugins/plugins

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/camera/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/clock/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/controlboard/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/externalwrench/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/forcetorque/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/imu/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/jointsensors/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/maissensor/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelzmp/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/videotexture/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/cmake_install.cmake")
  INCLUDE("/home/neda/iCube/gazebo-yarp-plugins/plugins/lasersensor/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

