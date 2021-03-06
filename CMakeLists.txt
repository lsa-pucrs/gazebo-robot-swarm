cmake_minimum_required(VERSION 2.8.3)
project(suporteformacao)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs tf)

#######################
## Gazebo Standalone ##
#######################

find_package(gazebo REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p2os_urdf
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(       sf_gazebo_bridge     src/sf_gazebo_bridge.cc)
target_link_libraries(sf_gazebo_bridge     ${catkin_LIBRARIES} 
                                        ${GAZEBO_LIBRARIES} 
                                        ${Boost_LIBRARIES} 
                                        ${PROTOBUF_LIBRARIES})

add_executable(       sf_gazebo_laserscan     src/sf_gazebo_laserscan.cc)
target_link_libraries(sf_gazebo_laserscan     ${catkin_LIBRARIES} 
                                        ${GAZEBO_LIBRARIES} 
                                        ${Boost_LIBRARIES} 
                                        ${PROTOBUF_LIBRARIES})

## Declare a cpp executable

#############
## Install ##
#############
install(PROGRAMS  scripts/amcl_init
                  scripts/teleop_joy
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS sf_gazebo_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
  urdf 
  launch
  meshes 
  models
  robots 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

