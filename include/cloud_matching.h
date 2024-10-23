#ifndef CLOUD_MATCHING_H
#define CLOUD_MATCHING_H

#include "utils.h"

class CloudMatching {
public:
    CloudMatching();
    ~CloudMatching();
    bool MatchNormalClouds(const KeyFrame & keyi, const KeyFrame & keyj, Eigen::Matrix4d & rel_poseT, double & residual_sum,
                           Eigen::Vector3d & eigenvalues, Eigen::Matrix3d & eigenvectors);

    bool MatchNormalCloudsNoDegeneracy(const KeyFrame & keyi, const KeyFrame & keyj, Eigen::Matrix4d & rel_poseT, double & residual_sum,
                                       Eigen::Vector3d & eigenvalues, Eigen::Matrix3d & eigenvectors);

    void TransformPoints(const PointsWithNormals & po, PointsWithNormals & pm, const Eigen::Matrix4d T);
    void SetParams(const float _nearest_point_range, 
                   const float _voxel_size, 
                   const float hor_fov_,
                   const float ver_max_,
                   const float ver_min_,
                   const int hor_pixel_num_,
                   const int ver_pixel_num_,
                   const Eigen::Matrix4d & lidar_to_imu_,
                   const Eigen::Matrix4d & imu_to_lidar_,
                   const int show_img_);

    bool show_img;

    float voxel_size;
    float hor_fov;
    float ver_max;
    float ver_min;
    int hor_pixel_num;
    int ver_pixel_num;

    float hor_resolution;
    float ver_resolution;

    Eigen::Matrix4d lidar_to_imu;
    Eigen::Matrix4d imu_to_lidar;

    bool MatchByProjection(const KeyFrame & keyi, 
                           const KeyFrame & keyj, 
                           Eigen::Matrix4d & rel_poseT, 
                           double & residual,
                           int & projected_no);

    // bool MatchByProjectionAlt(const KeyFrame & keyi, 
    //                        const KeyFrame & keyj, 
    //                        Eigen::Matrix4d & rel_poseT, 
    //                        double & residual,
    //                        int & projected_no);


    void ProjectCloud(const pcl::PointCloud<PointsWithNormals>::Ptr & cloud_int, 
                      pcl::PointCloud<PointsWithNormals>::Ptr & cloud_out);


private:
    pcl::VoxelGrid<PointsWithNormals> voxelGridFilter;
    pcl::VoxelGrid<PointsWithNormals> voxelGridFilterLoop;

    pcl::KdTreeFLANN<PointsWithNormals>::Ptr kdtreeNormalPoint;
    float nearest_point_range;

    void get_uv(const PointsWithNormals & point, int & u, int & v);
    float get_range(PointsWithNormals & pt);


    cv::Mat ori_submap_img;
    cv::Mat ori_matched_img;

    cv::Mat now_img;
    cv::Mat now_img_full;
    cv::Mat submap_img;
    cv::Mat matched_img;


};


#endif