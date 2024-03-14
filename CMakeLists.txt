cmake_minimum_required(VERSION 3.0.2)
project(AdaCt)

SET(CMAKE_BUILD_TYPE "Debug")


## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)

#add varirant
add_definitions(-DDEBUG_PRINT=\"FALSE\")

ADD_COMPILE_OPTIONS(-std=c++17 )
ADD_COMPILE_OPTIONS(-std=c++17 )
# set c++17 03等级，优化效率，这是关键！！
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
        message_generation
        tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP QUIET)



find_package(PCL REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(tsl-robin-map REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMp)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_message_files(
        FILES
        cloud_info.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES AdaCt
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs message_generation message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
)

add_executable(preprocess src/preprocess.cpp)
target_link_libraries(preprocess
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_executable(preprocess_new src/preprocess_new.cpp)
target_link_libraries(preprocess_new
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_executable(featureAssociation src/featureAssociation.cpp)
target_link_libraries(featureAssociation
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )


add_executable(lidar_odometry src/old/lidar_odometry.cpp include/ikd-Tree/ikd_Tree.cpp)
target_link_libraries(lidar_odometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${Sophus_LIBRARIES}
        tsl::robin_map
)

add_executable(lidar_odometry_new src/old/lidar_odometry_new.cpp include/ikd-Tree/ikd_Tree.cpp)
target_link_libraries(lidar_odometry_new
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        )

add_executable(lidar_odometry_downsample src/old/lidar_odometry_downsample.cpp include/ikd-Tree/ikd_Tree.cpp)
target_link_libraries(lidar_odometry_downsample
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        )

add_executable(lidar_odometry_voxelmap src/lidar_odometry_voxelmap.cpp )
target_link_libraries(lidar_odometry_voxelmap
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        #OpenMP::OpenMP_CXX
        ${OpenMP_CXX_FLAGS}
        )

add_executable(lidar_odometry_multiResolution_voxelmap src/lidar_odometry_multiResolution_voxelmap.cpp )
target_link_libraries(lidar_odometry_multiResolution_voxelmap
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        #OpenMP::OpenMP_CXX
        ${OpenMP_CXX_FLAGS}
        ${OpenCV_LIBRARIES}
        )

add_executable(lidar_odometry_trajectory src/lidar_odometry_trajectory.cpp )
target_link_libraries(lidar_odometry_trajectory
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        #OpenMP::OpenMP_CXX
        ${OpenMP_CXX_FLAGS}
        ${OpenCV_LIBRARIES}
        )


add_executable(lidar_odometry_voxelmap_segment src/lidar_odometry_voxelmap_segment.cpp )
target_link_libraries(lidar_odometry_voxelmap_segment
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        ${OpenCV_LIBRARIES}
        ${OpenMP_CXX_FLAGS}
        )


add_executable(lidar_odometry_voxelmapMultiMode src/lidar_odometry_voxelmapMultiMode.cpp)
target_link_libraries(lidar_odometry_voxelmapMultiMode
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        OpenMP::OpenMP_CXX
        )

add_executable(lidar_odometry_featureVoxelmap src/old/lidar_odometry_featureVoxelmap.cpp)
target_link_libraries(lidar_odometry_featureVoxelmap
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        OpenMP::OpenMP_CXX
        )

add_executable(lidar_odometry_featureVoxelmapNew src/old/lidar_odometry_featureVoxelmapNew.cpp)
target_link_libraries(lidar_odometry_featureVoxelmapNew
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        tsl::robin_map
        OpenMP::OpenMP_CXX
        )

add_executable(Transform src/Transform.cpp)
target_link_libraries(Transform
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_AdaCt.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
