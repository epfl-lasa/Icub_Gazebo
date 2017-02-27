# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_LoadURDF)

# Add for filtering data
find_package (Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)

include(AddGazeboYarpPluginTarget)


message(STATUS "libraries: "${iDynTree_LIBRARIES})

add_gazebo_yarp_plugin_target(LIBRARY_NAME loadurdf
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/gazebo 
                              SYSTEM_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} ${SG_DIFFERENTATION_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${iDynTree_LIBRARIES} sg_differentiation
                              HEADERS include/gazebo/LoadURDF.hh
                              SOURCES src/LoadURDF.cc
                                 )
