# Copyright: (C) 2010-2011 RobotCub Consortium
# Authors: Matteo Fumagalli, Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(iDyn)

set(folder_source src/iDyn.cpp
                  src/iDynInv.cpp
                  src/iDynBody.cpp
                  src/iDynTransform.cpp
                  src/iDynContact.cpp)

set(folder_header include/iCub/iDyn/iDyn.h
                  include/iCub/iDyn/iDynInv.h
                  include/iCub/iDyn/iDynBody.h
                  include/iCub/iDyn/iDynTransform.h
                  include/iCub/iDyn/iDynContact.h)

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)
add_library(${PROJECT_NAME} ${folder_source} ${folder_header})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${PROJECT_NAME} iKin
                                      skinDynLib
                                      ${YARP_LIBRARIES})

icub_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    DEPENDS iKin skinDynLib
                                    DESTINATION include/iCub/iDyn
                                    FILES ${folder_header})


