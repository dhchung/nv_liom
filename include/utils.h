#ifndef UTILS_H
#define UTILS_H

#define PCL_NO_PRECOMPILE

#include <iostream>
#include <signal.h>
#include <fstream>
#include <string.h>
#include <thread>
#include <mutex>
#include <deque>
#include <signal.h>
#include <dirent.h>
#include <unordered_map>


//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/publisher.h>
#include <pcl/filters/passthrough.h>


//Messages
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>

//GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/base/serialization.h>

//EIGEN
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

enum class SensorType { OUSTER, HESAI, VELODYNEXYZIRT, VELODYNE};

struct DeskewPoint {
    PCL_ADD_POINT4D;
    float intensity;
    double t;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(DeskewPoint,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (double, t, t) (std::uint32_t, range, range)
)


struct OusterPoint {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPoint,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint32_t, t, t) (std::uint32_t, range, range)
    
)

struct HesaiPoint {
  PCL_ADD_POINT4D
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    HesaiPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (double, timestamp, timestamp)(std::uint16_t, ring, ring)
)

// struct HesaiPoint {
//   PCL_ADD_POINT4D
//   float intensity;
//   double timestamp;
//   std::uint16_t ring;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(
//     HesaiPoint,
//     (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
//     (double, timestamp, timestamp)(std::uint16_t, ring, ring)
// )



struct VelodynePoint {
    PCL_ADD_POINT4D;
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
)

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)

)

struct LivoxPoint {
  PCL_ADD_POINT4D
  float intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    LivoxPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (double, timestamp, timestamp)
)

struct ProjectedPoint {
    PCL_ADD_POINT4D;
    float intensity;
    float range = 0.0f;
    int valid = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(ProjectedPoint,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, range, range) (int, valid, valid)
)

struct PointsWithNormals {
    PCL_ADD_POINT4D;
    PCL_ADD_NORMAL4D;
    float intensity;
    float range = 0.0f;
    int valid = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointsWithNormals,
    (float, x, x) (float, y, y) (float, z, z) (float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z)
    (float, intensity, intensity) (float, range, range) (int, valid, valid)
)

struct KeyFrame {
    pcl::PointCloud<PointsWithNormals>::Ptr normalPoints;
    pcl::PointCloud<PointsWithNormals>::Ptr totalPoints;
    gtsam::Pose3 pose;
    gtsam::Pose3 lidar_pose;
    double time;
    int node_no;
    bool is_stable;

    void InsertNormalPoints(const pcl::PointCloud<PointsWithNormals>::Ptr & normalPoints_) {
        normalPoints = normalPoints_->makeShared();
    }

    void InsertTotalPoints(const pcl::PointCloud<PointsWithNormals>::Ptr & totalPoints_) {
        totalPoints = totalPoints_->makeShared();
    }

    void AugmentNormalPoints(const pcl::PointCloud<PointsWithNormals>::Ptr & normalPoints_) {
        normalPoints->insert(normalPoints->end(), normalPoints_->begin(), normalPoints_->end());
    }

    KeyFrame() {
        normalPoints.reset(new pcl::PointCloud<PointsWithNormals>());
        totalPoints.reset(new pcl::PointCloud<PointsWithNormals>());
        is_stable = true;
    }
};

struct KeyframeLocation {
    PCL_ADD_POINT4D;
    float qx;
    float qy;
    float qz;
    float qw;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(KeyframeLocation,
    (float, x, x) (float, y, y) (float, z, z) (float, qx, qx) (float, qy, qy) (float, qz, qz) (float, qw, qw)
)

struct Coefficients{
    float x, y, z;
    float intensity;
    bool valid;

    Coefficients() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        intensity = 0.0;
        valid = false;
    }

};

#endif
