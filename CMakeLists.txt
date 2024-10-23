cmake_minimum_required(VERSION 3.0.2)
project(nv_liom)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED QUIET)            # Graph SLAM
find_package(OpenCV REQUIRED)                 # Image based localization
find_package(OpenMP REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
 geometry_msgs 
 nav_msgs 
 roscpp 
 sensor_msgs
 visualization_msgs 
 std_msgs 
 tf 
 tf2 
 pcl_conversions 
 pcl_ros 
 image_transport 
 cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/include/*.hpp
                    ${PROJECT_SOURCE_DIR}/classes/*.cpp)

add_executable(nv_lidar_deskew_node src/lidarDeskew.cpp)
  target_link_libraries(nv_lidar_deskew_node
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    gtsam
    OpenMP::OpenMP_CXX
)

add_executable(nv_lidar_projection_node src/lidarProjection.cpp)
  target_link_libraries(nv_lidar_projection_node
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    gtsam
    OpenMP::OpenMP_CXX
)

add_executable(nv_normal_extraction_node src/normalExtraction.cpp)
  target_link_libraries(nv_normal_extraction_node
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    gtsam
    OpenMP::OpenMP_CXX
)

add_executable(nv_lidar_inertial_mapping_node src/lidarInertialMapping.cpp ${SRC_FILES})
  target_link_libraries(nv_lidar_inertial_mapping_node
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    gtsam
    OpenMP::OpenMP_CXX
)
