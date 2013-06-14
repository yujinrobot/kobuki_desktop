cmake_minimum_required(VERSION 2.8.3)
project(kobuki_gazebo_plugins)

# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

find_package(catkin REQUIRED COMPONENTS gazebo_ros
                                        geometry_msgs
                                        kobuki_msgs
                                        nav_msgs
                                        roscpp
                                        std_msgs
                                        tf)

catkin_package(INCLUDE_DIRS include
               LIBRARIES gazebo_ros_kobuki
               CATKIN_DEPENDS gazebo_ros
                              geometry_msgs
                              kobuki_msgs
                              nav_msgs
                              roscpp
                              std_msgs
                              tf)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_ros_kobuki src/gazebo_ros_kobuki.cpp)

target_link_libraries(gazebo_ros_kobuki ${catkin_LIBRARIES})

install(TARGETS gazebo_ros_kobuki
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})