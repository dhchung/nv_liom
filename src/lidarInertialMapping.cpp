#include "utils.h"
#include "cloud_matching.h"

CloudMatching cm;

float voxel_size;
float vis_voxel_size;
float save_voxel_size;
int vis_node_skip;
float mapping_range;
float mapping_closest_range;
float mapping_node_distance;
int submap_size;
int mapping_in_rgb;
float gravity;

int hor_pixel_num;
int ver_pixel_num;
float hor_fov;
float ver_max;
float ver_min;
float hor_resolution;
float ver_resolution;
int loop_closed_history;
int min_gap;

Eigen::Vector3d LastLoopClosedPos;
// gtsam::Pose3 LastAugmentationPose;

std::string map_save_dir;
std::string cloud_save_dir;
std::string normal_cloud_save_dir;
std::string keyframe_pose_save_path;
std::string pose_save_path;
std::string lidar_pose_save_path;
std::string loop_index_save_path;

std::string graph_save_path;
std::string graph_values_save_path;
std::string keyframe_no_to_node_no_path;

std::mutex mtx;
std::deque<sensor_msgs::Imu> imuQueueOpt;
std::deque<sensor_msgs::Imu> imuQueueImu;

std::thread publishMapThread;
ros::Publisher pubCloud;
ros::Publisher pubCloudOptimized;

bool mapPublish;

std::vector<KeyFrame> PointNodes;
std::vector<std::pair<double, int>> nodeTimePairs;

int show_img;


using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

//IMU Preintegration
double imuAccNoise, imuGyrNoise, imuAccBiasN, imuGyrBiasN, imuGravity, correctionNoiseAng, correctionNoiseTrans;
gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise, priorVelNoise, priorBiasNoise, correctionNoise;
gtsam::Vector noiseModelBetweenBias;
gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

gtsam::Pose3 prevPose_;
gtsam::Vector3 prevVel_;
gtsam::NavState prevState_;
gtsam::imuBias::ConstantBias prevBias_;

gtsam::NavState prevStateOdom;
gtsam::imuBias::ConstantBias prevBiasOdom;

gtsam::ISAM2 optimizer;
gtsam::NonlinearFactorGraph factorGraph;
gtsam::Values graphValues;

gtsam::Values result;

ros::Publisher pubOdometry;
ros::Publisher pubImuOdometry;
ros::Publisher pubFactorEdge;
ros::Publisher pubCurrLoopClosure;
ros::Publisher pubLoopClosure;

ros::Publisher pubLoopCloud_curr;
ros::Publisher pubLoopCloud_target;
ros::Publisher pubLoopCurrOdometry;
ros::Publisher pubLoopTargetOdometry;

ros::Publisher pubAngularBias;

pcl::PointCloud<PointsWithNormals>::Ptr mapPoints;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPoints_rgb;

std::vector<std::pair<int, int>> factorEdges;
std::vector<std::pair<int, int>> loopEdges;

Eigen::Matrix3d lidar_to_imu_R;
Eigen::Vector3d lidar_to_imu_t;

Eigen::Matrix4d lidar_to_imu_T;
Eigen::Matrix4d imu_to_lidar_T;

Eigen::Matrix4d initial_pose;
KeyFrame curr_keyframe;

pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeKeyFrame;

//Save Factors

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,"gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base, "gtsam_noiseModel_mEstimator_Base");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null,"gtsam_noiseModel_mEstimator_Null");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair, "gtsam_noiseModel_mEstimator_Fair");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber,"gtsam_noiseModel_mEstimator_Huber");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");
GTSAM_VALUE_EXPORT(gtsam::Point3);
GTSAM_VALUE_EXPORT(gtsam::Rot3);
GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::imuBias::ConstantBias);
GTSAM_VALUE_EXPORT(gtsam::NavState);
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor");

// Add your custom factors, if any.
BOOST_CLASS_EXPORT_GUID(gtsam::ImuFactor, "gtsam::ImuFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>, "gtsam::BetweenFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Point3>, "gtsam::BetweenFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Rot3>, "gtsam::BetweenFactor<gtsam::Rot3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>, "gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>");

BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>, "gtsam::PriorFactor<gtsam::imuBias::ConstantBias>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Point3>, "gtsam::PriorFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Rot3>, "gtsam::PriorFactor<gtsam::Rot3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::Values, "gtsam::Values");
BOOST_CLASS_EXPORT_GUID(gtsam::NonlinearFactorGraph, "gtsam::NonlinearFactorGraph");
BOOST_CLASS_EXPORT_GUID(gtsam::FactorGraph<gtsam::NonlinearFactor>, "gtsam::FactorGraph<gtsam::NonlinearFactor>");


std::vector<gtsam::PriorFactor<gtsam::Pose3>> vec_priorPoseFactors;
std::vector<gtsam::PriorFactor<gtsam::Vector3>> vec_priorVelFactors;
std::vector<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>> vec_priorBiasFactors;
std::vector<gtsam::BetweenFactor<gtsam::Pose3>> vec_betweenFactorsPoses;
std::vector<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>> vec_betweenFactorsBias;
std::vector<gtsam::ImuFactor> vec_imuFactors;
gtsam::Values vec_graphValues;

int key;


bool doneFirstOpt;

double lastImuOptTime;
double lastImuMsgTime;

std::vector<KeyFrame> nodesMap;
pcl::VoxelGrid<PointsWithNormals> voxelGrid;

pcl::VoxelGrid<PointsWithNormals> voxelGridFilterMap;

pcl::VoxelGrid<PointsWithNormals> voxelGridFilterVis;
pcl::VoxelGrid<pcl::PointXYZRGB> voxelGridFilterVis_rgb;


void PublishMapThread() {

    ros::Rate rate(1);
    while (ros::ok())
    {
        rate.sleep();
        mtx.lock();
        if(!mapPublish) {
            mtx.unlock();
            break;
        }

        if(PointNodes.empty()) {
            mtx.unlock();
            continue;
        }


        nodesMap = PointNodes;
        mtx.unlock();

        mapPoints.reset(new pcl::PointCloud<PointsWithNormals>());
        mapPoints_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::PointCloud<PointsWithNormals>::Ptr transformedPoints(new pcl::PointCloud<PointsWithNormals>());
        for(int i = 0; i < nodesMap.size(); i+=vis_node_skip) {
            Eigen::Matrix4d poseT = nodesMap[i].lidar_pose.matrix();
            if(mapping_in_rgb == 1) {
                pcl::transformPointCloudWithNormals(*nodesMap[i].normalPoints, *transformedPoints, poseT);
            } else {
                pcl::transformPointCloud(*nodesMap[i].totalPoints, *transformedPoints, poseT);
            }

            mapPoints->insert(mapPoints->end(), transformedPoints->begin(), transformedPoints->end());
        }

        if(mapping_in_rgb == 1) {
            for(int i = 0; i < mapPoints->size(); ++i) {
                pcl::PointXYZRGB pt;
                pt.x = mapPoints->points[i].x;
                pt.y = mapPoints->points[i].y;
                pt.z = mapPoints->points[i].z;
                pt.r = uint8_t(255.0 * (mapPoints->points[i].normal_x*0.5+0.5));
                pt.g = uint8_t(255.0 * (mapPoints->points[i].normal_y*0.5+0.5));
                pt.b = uint8_t(255.0 * (mapPoints->points[i].normal_z*0.5+0.5));
                mapPoints_rgb->push_back(pt);
            }
            
            if(vis_voxel_size > 0.1) {
                voxelGridFilterVis_rgb.setInputCloud(mapPoints_rgb);
                voxelGridFilterVis_rgb.filter(*mapPoints_rgb);
            }


            sensor_msgs::PointCloud2 temp_cloud;
            pcl::toROSMsg(*mapPoints_rgb, temp_cloud);
            temp_cloud.header.stamp = ros::Time::now();
            temp_cloud.header.frame_id = "map";
            pubCloud.publish(temp_cloud);
        } else {
            if(vis_voxel_size > 0.1) {
                voxelGridFilterVis.setInputCloud(mapPoints);
                voxelGridFilterVis.filter(*mapPoints);
            }

            sensor_msgs::PointCloud2 temp_cloud;
            pcl::toROSMsg(*mapPoints, temp_cloud);
            temp_cloud.header.stamp = ros::Time::now();
            temp_cloud.header.frame_id = "map";
            pubCloud.publish(temp_cloud);
        }
    }
}

void OnSubscribeIMU(const sensor_msgs::ImuConstPtr & msg) {
    std::lock_guard<std::mutex> lock(mtx);
    imuQueueOpt.push_back(*msg);
    imuQueueImu.push_back(*msg);

    if(!doneFirstOpt) {
        return;
    }

    double imuTime = msg->header.stamp.toSec();
    double dt = (lastImuMsgTime < 0) ? (1.0/400.0) : (imuTime - lastImuMsgTime);
    lastImuMsgTime = imuTime;
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                                            gtsam::Vector3(msg->angular_velocity.x,    msg->angular_velocity.y,    msg->angular_velocity.z), dt);

    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
    nav_msgs::Odometry odometry;
    odometry.header.stamp = msg->header.stamp;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "lidar";
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());

    gtsam::Pose3 lidarPose = imuPose * gtsam::Pose3(imu_to_lidar_T);
    
    odometry.pose.pose.position.x = imuPose.translation().x();
    odometry.pose.pose.position.y = imuPose.translation().y();
    odometry.pose.pose.position.z = imuPose.translation().z();
    odometry.pose.pose.orientation.x = imuPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = imuPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = imuPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = imuPose.rotation().toQuaternion().w();
    
    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = msg->angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = msg->angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = msg->angular_velocity.z + prevBiasOdom.gyroscope().z();
    pubImuOdometry.publish(odometry);

    static tf::TransformBroadcaster tfbr;
    tf::Transform tfT;
    tf::poseMsgToTF(odometry.pose.pose, tfT);
    tfbr.sendTransform(tf::StampedTransform(tfT, msg->header.stamp, "map", "imu"));


    geometry_msgs::Pose lidar_pose;
    lidar_pose.position.x = lidarPose.translation().x();
    lidar_pose.position.y = lidarPose.translation().y();
    lidar_pose.position.z = lidarPose.translation().z();
    lidar_pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    lidar_pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    lidar_pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    lidar_pose.orientation.w = lidarPose.rotation().toQuaternion().w();

    tf::poseMsgToTF(lidar_pose, tfT);
    tfbr.sendTransform(tf::StampedTransform(tfT, msg->header.stamp, "map", "lidar"));


}

void OnSavePointCloud(const int no, const KeyFrame & keyframe) {
    char keyframe_no[256];
    sprintf(keyframe_no, "%06d", no);

    std::string total_cloud_path = cloud_save_dir + "/" + std::string(keyframe_no) + ".bin";
    std::ofstream binfile(total_cloud_path.c_str(), std::ios::out | std::ios::binary);
    for(int pt_idx = 0; pt_idx < keyframe.totalPoints->size(); ++pt_idx) {
        binfile.write((char*)&keyframe.totalPoints->points[pt_idx].x, sizeof(float));
        binfile.write((char*)&keyframe.totalPoints->points[pt_idx].y, sizeof(float));
        binfile.write((char*)&keyframe.totalPoints->points[pt_idx].z, sizeof(float));
        binfile.write((char*)&keyframe.totalPoints->points[pt_idx].intensity, sizeof(float));
    }
    binfile.close();
}


void OnSaveNormalPointCloud(const int no, const KeyFrame & keyframe) {
    char keyframe_no[256];
    sprintf(keyframe_no, "%06d", no);

    std::string normal_cloud_path = normal_cloud_save_dir + "/" + std::string(keyframe_no) + ".bin";
    std::ofstream binfile(normal_cloud_path.c_str(), std::ios::out | std::ios::binary);

    for(int pt_idx = 0; pt_idx < keyframe.normalPoints->size(); ++pt_idx) {
        binfile.write((char*)&keyframe.normalPoints->points[pt_idx].x, sizeof(float));
        binfile.write((char*)&keyframe.normalPoints->points[pt_idx].y, sizeof(float));
        binfile.write((char*)&keyframe.normalPoints->points[pt_idx].z, sizeof(float));
        binfile.write((char*)&keyframe.normalPoints->points[pt_idx].intensity, sizeof(float));
        binfile.write((char*)&keyframe.normalPoints->points[pt_idx].normal_x, sizeof(float));
        binfile.write((char*)&keyframe.normalPoints->points[pt_idx].normal_y, sizeof(float));
        binfile.write((char*)&keyframe.normalPoints->points[pt_idx].normal_z, sizeof(float));
    }
    binfile.close();
}

void OnSubscribeNormalCloud(const sensor_msgs::PointCloud2ConstPtr & msg) {
    ++loop_closed_history;

    double cloud_time = msg->header.stamp.toSec();
    std::deque<sensor_msgs::Imu> lastToPointTimeIMU;

    mtx.lock();
    if(imuQueueOpt.empty()) {
        return;
    }
    if(PointNodes.empty()) {
        while(imuQueueOpt.front().header.stamp.toSec() < cloud_time) {
            imuQueueOpt.pop_front();
            if(imuQueueOpt.empty()) {
                break;
            }
        }
    } else {
        while(imuQueueOpt.front().header.stamp.toSec() < cloud_time) {
            lastToPointTimeIMU.push_back(imuQueueOpt.front());
            imuQueueOpt.pop_front();
            if(imuQueueOpt.empty()) {
                break;
            }
        }
    }

    while(imuQueueImu.front().header.stamp.toSec() < cloud_time) {
        imuQueueImu.pop_front();
        if(imuQueueImu.empty()) {
            break;
        }
    }

    mtx.unlock();

    pcl::PointCloud<PointsWithNormals>::Ptr normal_cloud_in(new pcl::PointCloud<PointsWithNormals>());
    pcl::PointCloud<PointsWithNormals>::Ptr normal_cloud(new pcl::PointCloud<PointsWithNormals>());
    pcl::fromROSMsg(*msg, *normal_cloud_in);

    if(normal_cloud_in->empty()) {
        printf("Empty Cloud\n");
        return;
    }

    for(int i = 0; i < normal_cloud_in->size(); ++i) {
        if(normal_cloud_in->points[i].valid == 1) {
            normal_cloud->push_back(normal_cloud_in->points[i]);
        }
    }

    curr_keyframe.InsertNormalPoints(normal_cloud);
    curr_keyframe.InsertTotalPoints(normal_cloud_in);
    curr_keyframe.node_no = key;
    curr_keyframe.pose = prevPose_;
    curr_keyframe.lidar_pose = curr_keyframe.pose * gtsam::Pose3(imu_to_lidar_T);
    curr_keyframe.time = msg->header.stamp.toSec();


    std::vector<std::pair<gtsam::Pose3, gtsam::Pose3>> curr_loopEdges;

    bool loop_closed = false;


    if(PointNodes.empty()) {
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);


        factorGraph.add(priorPose);
        factorGraph.add(priorVel);
        factorGraph.add(priorBias);

        vec_priorPoseFactors.push_back(priorPose);
        vec_priorVelFactors.push_back(priorVel);
        vec_priorBiasFactors.push_back(priorBias);


        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);

        nodeTimePairs.emplace_back(msg->header.stamp.toSec(), 0);

        vec_graphValues.insert(X(0), prevPose_);
        vec_graphValues.insert(V(0), prevVel_);
        vec_graphValues.insert(B(0), prevBias_);

        optimizer.update(factorGraph, graphValues);
        factorGraph.resize(0);
        graphValues.clear();
        
        if(save_voxel_size > 0.1) {
            voxelGrid.setInputCloud(curr_keyframe.totalPoints);
            voxelGrid.filter(*curr_keyframe.totalPoints);
        }
        PointNodes.push_back(curr_keyframe);
        result = optimizer.calculateEstimate();

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        lastImuOptTime = cloud_time;

        LastLoopClosedPos = prevPose_.translation().matrix();
        // LastAugmentationPose = prevPose_;
        ++key;

    } else {
        
        bool imu_not_ready = lastToPointTimeIMU.empty();
        bool imu_fail = false;

        //ImuPreintegration
        while(!lastToPointTimeIMU.empty()) {
            sensor_msgs::Imu *thisImu = & lastToPointTimeIMU.front();
            double imuTime = thisImu->header.stamp.toSec();
            double dt = (lastImuOptTime < 0) ? (1.0/400.0f) : (imuTime - lastImuOptTime);

            if(dt > 0.5) {
                imu_fail = true;
            }
            imuIntegratorOpt_->integrateMeasurement(
                gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            lastImuOptTime = imuTime;
            lastToPointTimeIMU.pop_front();
        }

        if(imu_not_ready) {
            return;
        }
        gtsam::NavState propState_;
        if(imu_fail) {
            propState_ = prevState_;
        } else {
            propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        }

        gtsam::Pose3 predictedPose = propState_.pose();
        curr_keyframe.pose = predictedPose;
        curr_keyframe.lidar_pose = predictedPose * gtsam::Pose3(imu_to_lidar_T);


        double dist_avg = 0.0;
        for(size_t i = 0; i < normal_cloud->size(); ++i) {
            dist_avg += std::sqrt(normal_cloud->points[i].x * normal_cloud->points[i].x + 
                                  normal_cloud->points[i].y * normal_cloud->points[i].y + 
                                  normal_cloud->points[i].z * normal_cloud->points[i].z);
        }
        dist_avg /= static_cast<double>(normal_cloud->size());

        bool match_exists = false;

        int previous_node_id = static_cast<int>(PointNodes.size()) - 1;

        KeyFrame submap;
        pcl::PointCloud<PointsWithNormals>::Ptr sub_cloud(new pcl::PointCloud<PointsWithNormals>());
        sub_cloud->insert(sub_cloud->end(), PointNodes[previous_node_id].normalPoints->begin(), PointNodes[previous_node_id].normalPoints->end());

        for(int j = 1; j < submap_size; ++j) {
            if(previous_node_id - j < 0) {
                break;
            }
            Eigen::Matrix4d rel_T = lidar_to_imu_T * PointNodes[previous_node_id].pose.matrix().inverse() * PointNodes[previous_node_id-j].pose.matrix() * imu_to_lidar_T;
            pcl::PointCloud<PointsWithNormals>::Ptr translated_cloud(new pcl::PointCloud<PointsWithNormals>());
            pcl::transformPointCloudWithNormals(*PointNodes[previous_node_id-j].normalPoints, *translated_cloud, rel_T);
            sub_cloud->insert(sub_cloud->end(), translated_cloud->begin(), translated_cloud->end());
        }


        Eigen::Matrix4d rel_poseT;
        submap.InsertNormalPoints(sub_cloud);
        submap.pose = PointNodes[previous_node_id].pose;
        submap.lidar_pose = submap.pose * gtsam::Pose3(imu_to_lidar_T);

        double residual_sum;

        Eigen::Vector3d eigenvalues;
        Eigen::Matrix3d eigenvectors;

        bool matched = cm.MatchNormalClouds(submap, curr_keyframe, rel_poseT, residual_sum, eigenvalues, eigenvectors);


        rel_poseT = imu_to_lidar_T * rel_poseT * lidar_to_imu_T;

        bool not_stable = false;

        if(matched) {
            match_exists = true;
            gtsam::Pose3 rel_pose(rel_poseT);
            if(eigenvalues(0) < 0.01) {

                not_stable = true;
                gtsam::noiseModel::Gaussian::shared_ptr gaussian_noise;
                Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6);
                Q*= 0.01;
                Q.block(3, 3, 3, 3) = eigenvectors * eigenvalues.cwiseInverse().asDiagonal() * eigenvectors.transpose() * 0.01;

                gaussian_noise = gtsam::noiseModel::Gaussian::Covariance(Q);
                gtsam::BetweenFactor<gtsam::Pose3> pose_factor(X(key), X(PointNodes[previous_node_id].node_no), rel_pose, gaussian_noise);

                printf("Eigenvalue: %f\n", eigenvalues(0));
                std::cout<<"eigenvalues"<<std::endl;
                std::cout<<eigenvalues<<std::endl;
                // std::cout<<"eigenvectors"<<std::endl;
                // std::cout<<eigenvectors<<std::endl;
                // std::cout<<"Q"<<std::endl;
                // std::cout<<Q<<std::endl;

                factorGraph.add(pose_factor);
                vec_betweenFactorsPoses.push_back(pose_factor);

                std::pair<int, int> factor_edge{key, PointNodes[previous_node_id].node_no};
                factorEdges.push_back(factor_edge);

            } else {
                gtsam::BetweenFactor<gtsam::Pose3> pose_factor(X(key), X(PointNodes[previous_node_id].node_no), rel_pose, correctionNoise);
                factorGraph.add(pose_factor);
                vec_betweenFactorsPoses.push_back(pose_factor);

                std::pair<int, int> factor_edge{key, PointNodes[previous_node_id].node_no};
                factorEdges.push_back(factor_edge);
            }

        }

        std::vector<int> candidate_loop;
        Eigen::Vector3d fromLastLoopPos = prevPose_.translation().matrix() - LastLoopClosedPos;

        if(loop_closed_history < 5 || fromLastLoopPos.norm() < 2.0) {

        } else {
            kdtreeKeyFrame.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

            pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_loc(new pcl::PointCloud<pcl::PointXYZ>);



            for(int i = 0; i < PointNodes.size(); ++i) {
                if((static_cast<int>(PointNodes.size()) - i) < min_gap) {
                    break;
                }
                pcl::PointXYZ loc(PointNodes[i].pose.translation().x(), 
                                PointNodes[i].pose.translation().y(), 
                                PointNodes[i].pose.translation().z());
                keyframe_loc->push_back(loc);
            }
            std::vector<int> knn_index;
            std::vector<float> knn_distance;

            if(!keyframe_loc->empty()) {
                kdtreeKeyFrame->setInputCloud(keyframe_loc);
                pcl::PointXYZ curr_loc(predictedPose.translation().x(), 
                                    predictedPose.translation().y(),
                                    predictedPose.translation().z());

                kdtreeKeyFrame->nearestKSearch(curr_loc, 1, knn_index, knn_distance);
            }

            if(!knn_index.empty()) {
                if(std::sqrt(knn_distance[0]) < mapping_range) {
                    candidate_loop.push_back(knn_index[0]);
                } else {
                    printf("No Loop Candidates - Distance of node %d: %f / %f\n", knn_index[0], std::sqrt(knn_distance[0]), mapping_range);
                }
            } else {
                printf("No Loop Candidates - Gap: %d\n", min_gap);
            }
            loop_closed_history = 0;
        }

        for(int i = 0; i < candidate_loop.size(); ++i) {
            int candid_node_id = candidate_loop[i];

            Eigen::Matrix4d rel_poseT_loop;
            Eigen::Matrix4d rel_poseT_loop_lidar;

            double residual_loop = 0;

            int projected_no;

            bool loop_matched = cm.MatchByProjection(PointNodes[candid_node_id], 
                                                     curr_keyframe, 
                                                     rel_poseT_loop, 
                                                     residual_loop, 
                                                     projected_no);

            rel_poseT_loop_lidar = rel_poseT_loop;
            rel_poseT_loop = imu_to_lidar_T * rel_poseT_loop * lidar_to_imu_T;

            curr_loopEdges.emplace_back(PointNodes[candid_node_id].pose, predictedPose);
            if(loop_matched) {
                loop_closed = true;
                gtsam::Pose3 rel_pose(rel_poseT_loop);
                gtsam::BetweenFactor<gtsam::Pose3> pose_factor(X(key),X(PointNodes[candid_node_id].node_no), rel_pose, correctionNoise);
                factorGraph.add(pose_factor);
                vec_betweenFactorsPoses.push_back(pose_factor);

                std::pair<int, int> factor_edge{key, PointNodes[candid_node_id].node_no};
                loopEdges.push_back(factor_edge);

                if(show_img == 1) {
                    pcl::PointCloud<PointsWithNormals>::Ptr curr_loopCloud(new pcl::PointCloud<PointsWithNormals>(*curr_keyframe.totalPoints));
                    pcl::PointCloud<PointsWithNormals>::Ptr curr_targetCloud(new pcl::PointCloud<PointsWithNormals>(*PointNodes[candid_node_id].totalPoints));
                    pcl::transformPointCloudWithNormals(*curr_targetCloud, *curr_targetCloud, rel_poseT_loop_lidar);

                    sensor_msgs::PointCloud2 loop_cloud_msg;

                    pcl::toROSMsg(*curr_loopCloud, loop_cloud_msg);
                    loop_cloud_msg.header = msg->header;
                    pubLoopCloud_curr.publish(loop_cloud_msg);

                    pcl::toROSMsg(*curr_targetCloud, loop_cloud_msg);
                    loop_cloud_msg.header = msg->header;
                    pubLoopCloud_target.publish(loop_cloud_msg);

                    nav_msgs::Odometry odometry;
                    odometry.header.stamp = msg->header.stamp;
                    odometry.header.frame_id = "map";
                    odometry.child_frame_id = "lidar";
                    
                    odometry.pose.pose.position.x = curr_keyframe.lidar_pose.translation().x();
                    odometry.pose.pose.position.y = curr_keyframe.lidar_pose.translation().y();
                    odometry.pose.pose.position.z = curr_keyframe.lidar_pose.translation().z();
                    odometry.pose.pose.orientation.x = curr_keyframe.lidar_pose.rotation().toQuaternion().x();
                    odometry.pose.pose.orientation.y = curr_keyframe.lidar_pose.rotation().toQuaternion().y();
                    odometry.pose.pose.orientation.z = curr_keyframe.lidar_pose.rotation().toQuaternion().z();
                    odometry.pose.pose.orientation.w = curr_keyframe.lidar_pose.rotation().toQuaternion().w();
                    
                    pubLoopCurrOdometry.publish(odometry);

                    odometry.pose.pose.position.x = PointNodes[candid_node_id].lidar_pose.translation().x();
                    odometry.pose.pose.position.y = PointNodes[candid_node_id].lidar_pose.translation().y();
                    odometry.pose.pose.position.z = PointNodes[candid_node_id].lidar_pose.translation().z();
                    odometry.pose.pose.orientation.x = PointNodes[candid_node_id].lidar_pose.rotation().toQuaternion().x();
                    odometry.pose.pose.orientation.y = PointNodes[candid_node_id].lidar_pose.rotation().toQuaternion().y();
                    odometry.pose.pose.orientation.z = PointNodes[candid_node_id].lidar_pose.rotation().toQuaternion().z();
                    odometry.pose.pose.orientation.w = PointNodes[candid_node_id].lidar_pose.rotation().toQuaternion().w();
                    
                    pubLoopTargetOdometry.publish(odometry);
                }
            }
        }

        if(match_exists || loop_closed) {
            if(!doneFirstOpt) {
                doneFirstOpt = true;
            }

            const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
            gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);

            gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> bias_factor(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias));

            factorGraph.add(imu_factor);
            factorGraph.add(bias_factor);

            vec_imuFactors.push_back(imu_factor);
            vec_betweenFactorsBias.push_back(bias_factor);

            graphValues.insert(X(key), predictedPose);
            graphValues.insert(V(key), propState_.v());
            graphValues.insert(B(key), prevBias_);

            nodeTimePairs.emplace_back(msg->header.stamp.toSec(), key);

            vec_graphValues.insert(X(key), predictedPose);
            vec_graphValues.insert(V(key), propState_.v());
            vec_graphValues.insert(B(key), prevBias_);


            optimizer.update(factorGraph, graphValues);

            factorGraph.resize(0);
            graphValues.clear();

            result = optimizer.calculateEstimate();
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_   = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            prevStateOdom = prevState_;
            prevBiasOdom  = prevBias_;

            std_msgs::Float64MultiArray bias_msg;
            bias_msg.data = std::vector<double>{prevBias_.gyroscope().x(), prevBias_.gyroscope().y(), prevBias_.gyroscope().z()};
            pubAngularBias.publish(bias_msg);

            if(loop_closed) {
                LastLoopClosedPos = prevPose_.translation().matrix();
            }

            //Keyframe Insertion
            gtsam::Pose3 rel_pose = PointNodes.back().pose.between(prevPose_);
            Eigen::AngleAxisd rel_rot(rel_pose.rotation().matrix());

            if(rel_pose.translation().norm() > mapping_node_distance || rel_rot.angle() > 30*M_PI/180.0f) {
            // if(rel_pose.translation().norm() > mapping_node_distance) {
                if(save_voxel_size > 0.1) {
                    voxelGrid.setInputCloud(curr_keyframe.totalPoints);
                    voxelGrid.filter(*curr_keyframe.totalPoints);
                }
                curr_keyframe.is_stable = !not_stable;

                PointNodes.push_back(curr_keyframe);
                // LastAugmentationPose = prevPose_;

            }

            //Update Poses
            for(int i = 0; i < PointNodes.size(); ++i) {
                PointNodes[i].pose = result.at<gtsam::Pose3>(X(PointNodes[i].node_no));
                PointNodes[i].lidar_pose = PointNodes[i].pose * gtsam::Pose3(imu_to_lidar_T);
            }

            ++key;

            if (!imuQueueImu.empty())
            {
                imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);

                double lastImuQT = cloud_time;
                for (int i = 0; i < (int)imuQueueImu.size(); ++i)
                {
                    sensor_msgs::Imu *thisImu = &imuQueueImu[i];
                    double imuTime = thisImu->header.stamp.toSec();;
                    double dt = (lastImuQT < 0) ? (1.0 / 100.0) :(imuTime - lastImuQT);
                    if(dt <= 0) {
                        dt = 0.01;
                    }

                    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                            gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                    lastImuQT = imuTime;
                }
            }
        } else {
            printf("\x1b[31mMatching Failed\x1b[0m\n");
        }
    }

    nav_msgs::Odometry odometry;
    odometry.header.stamp = msg->header.stamp;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "lidar";

    odometry.pose.pose.position.x = prevPose_.translation().x();
    odometry.pose.pose.position.y = prevPose_.translation().y();
    odometry.pose.pose.position.z = prevPose_.translation().z();
    odometry.pose.pose.orientation.x = prevPose_.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = prevPose_.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = prevPose_.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = prevPose_.rotation().toQuaternion().w();

    pubOdometry.publish(odometry);

    gtsam::Pose3 lidarPose = prevPose_ * gtsam::Pose3(imu_to_lidar_T);

    geometry_msgs::Pose lidar_pose;
    lidar_pose.position.x = lidarPose.translation().x();
    lidar_pose.position.y = lidarPose.translation().y();
    lidar_pose.position.z = lidarPose.translation().z();
    lidar_pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    lidar_pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    lidar_pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    lidar_pose.orientation.w = lidarPose.rotation().toQuaternion().w();

    static tf::TransformBroadcaster tfbr;
    tf::Transform tfT;
    tf::poseMsgToTF(lidar_pose, tfT);
    tfbr.sendTransform(tf::StampedTransform(tfT, msg->header.stamp, "map", "lidar"));

    // sensor_msgs::PointCloud2 normal_cloud_msg;
    // pcl::toROSMsg(*normal_cloud, normal_cloud_msg);
    // normal_cloud_msg.header = msg->header;
    pubCloudOptimized.publish(*msg);


    // factor edges
    visualization_msgs::MarkerArray markerArrayTotal;
    visualization_msgs::Marker markerEdgeTotal;
    markerEdgeTotal.header.frame_id = "map";
    markerEdgeTotal.header.stamp = msg->header.stamp;
    markerEdgeTotal.action = visualization_msgs::Marker::ADD;
    markerEdgeTotal.type = visualization_msgs::Marker::LINE_LIST;
    markerEdgeTotal.ns = "factor_edges";
    markerEdgeTotal.id = 1;
    markerEdgeTotal.pose.orientation.w = 1;
    markerEdgeTotal.scale.x = 0.05;
    markerEdgeTotal.color.r = 1.0; markerEdgeTotal.color.g = 132.0/255.0; markerEdgeTotal.color.b = 0;
    markerEdgeTotal.color.a = 1;

    for(int i = 1; i < key; ++i) {
        gtsam::Pose3 curr_pose = result.at<gtsam::Pose3>(X(i));
        gtsam::Pose3 prev_pose = result.at<gtsam::Pose3>(X(i-1));
        geometry_msgs::Point p;
        p.x = prev_pose.translation().x();
        p.y = prev_pose.translation().y();
        p.z = prev_pose.translation().z();
        markerEdgeTotal.points.push_back(p);
        p.x = curr_pose.translation().x();
        p.y = curr_pose.translation().y();
        p.z = curr_pose.translation().z();
        markerEdgeTotal.points.push_back(p);
    }

    markerArrayTotal.markers.push_back(markerEdgeTotal);
    pubFactorEdge.publish(markerArrayTotal);


    // loop edges
    visualization_msgs::MarkerArray markerArrayLoop;
    visualization_msgs::Marker markerEdgeLoop;
    markerEdgeLoop.header.frame_id = "map";
    markerEdgeLoop.header.stamp = msg->header.stamp;
    markerEdgeLoop.action = visualization_msgs::Marker::ADD;
    markerEdgeLoop.type = visualization_msgs::Marker::LINE_LIST;
    markerEdgeLoop.ns = "loop_edges";
    markerEdgeLoop.id = 2;
    markerEdgeLoop.pose.orientation.w = 1;
    markerEdgeLoop.scale.x = 0.05;
    markerEdgeLoop.color.r = 0.0; markerEdgeLoop.color.g = 175.0/255.0; markerEdgeLoop.color.b = 240.0/255.0;
    markerEdgeLoop.color.a = 1;

    for(int i = 0; i < loopEdges.size(); ++i) {
        int key_pre = loopEdges[i].first;
        int key_cur = loopEdges[i].second;
        geometry_msgs::Point p;
        gtsam::Pose3 prev_pose = result.at<gtsam::Pose3>(X(key_pre));
        gtsam::Pose3 curr_pose = result.at<gtsam::Pose3>(X(key_cur));
        p.x = prev_pose.translation().x();
        p.y = prev_pose.translation().y();
        p.z = prev_pose.translation().z();
        markerEdgeLoop.points.push_back(p);
        p.x = curr_pose.translation().x();
        p.y = curr_pose.translation().y();
        p.z = curr_pose.translation().z();
        markerEdgeLoop.points.push_back(p);
    }

    markerArrayLoop.markers.push_back(markerEdgeLoop);
    pubLoopClosure.publish(markerArrayLoop);


    visualization_msgs::MarkerArray markerArrayCurrLoop;
    visualization_msgs::Marker markerCurrEdgeLoop;
    markerCurrEdgeLoop.header.frame_id = "map";
    markerCurrEdgeLoop.header.stamp = msg->header.stamp;
    markerCurrEdgeLoop.action = visualization_msgs::Marker::ADD;
    markerCurrEdgeLoop.type = visualization_msgs::Marker::LINE_LIST;
    markerCurrEdgeLoop.ns = "curr_loop_edges";
    markerCurrEdgeLoop.id = 3;
    markerCurrEdgeLoop.pose.orientation.w = 1;
    markerCurrEdgeLoop.scale.x = 0.1;
    markerCurrEdgeLoop.color.a = 1;
    markerCurrEdgeLoop.color.r = 60.0/255.0; markerCurrEdgeLoop.color.g = 144.0/255.0; markerCurrEdgeLoop.color.b = 86.0/255.0;

    if(loopEdges.empty()) {

    } else {
        int key_pre = loopEdges.back().first;
        int key_cur = loopEdges.back().second;
        gtsam::Pose3 prev_pose = result.at<gtsam::Pose3>(X(key_pre));
        gtsam::Pose3 curr_pose = result.at<gtsam::Pose3>(X(key_cur));

        geometry_msgs::Point p;
        p.x = prev_pose.translation().x();
        p.y = prev_pose.translation().y();
        p.z = prev_pose.translation().z();
        markerCurrEdgeLoop.points.push_back(p);
        p.x = curr_pose.translation().x();
        p.y = curr_pose.translation().y();
        p.z = curr_pose.translation().z();
        markerCurrEdgeLoop.points.push_back(p);

        markerArrayCurrLoop.markers.push_back(markerCurrEdgeLoop);
        pubCurrLoopClosure.publish(markerArrayCurrLoop);
    }

}

void GraphParamInitialization() {

    imuGravity = gravity;

    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2);
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2);
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

    priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 5e-2, 5e-2, 5e-2, 5e-2, 5e-2, 5e-2).finished());
    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
    priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
    correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01).finished());
    noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
    
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
}

void resetGraph() {
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newFactorGraph;
    factorGraph = newFactorGraph;

    gtsam::Values newValues;
    graphValues = newValues;

    prevPose_ = gtsam::Pose3(initial_pose);
    prevVel_ = gtsam::Vector3(0.0, 0.0, 0.0);
    prevBias_ = gtsam::imuBias::ConstantBias();

    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevStateOdom = prevState_;
    prevBiasOdom = prevBias_;
    GraphParamInitialization();

}

void OnReceiveEndMappingMessage(const std_msgs::BoolConstPtr & msg) {
    printf("Save Map Received\n");
    printf("Checking Directories\n");

    if(map_save_dir.empty()) {
        map_save_dir = std::getenv("HOME") + std::string("/Download/NV_LIOM");
    }
    printf("Saving in :%s\n", map_save_dir.c_str());
    int unused = system((std::string("exec rm -r ") + cloud_save_dir).c_str());
    unused = system((std::string("exec rm -r ") + normal_cloud_save_dir).c_str());
    unused = system((std::string("mkdir -p ") + map_save_dir).c_str());
    unused = system((std::string("mkdir -p ") + cloud_save_dir).c_str());
    unused = system((std::string("mkdir -p ") + normal_cloud_save_dir).c_str());

    if(!msg->data) {
        return;
    }
    mtx.lock();
    mapPublish = false;
    mtx.unlock();

    if(publishMapThread.joinable()) {
        publishMapThread.join();
    }

    if(PointNodes.empty()) {
        printf("No Data To Save\n");
        ros::shutdown();
    } else {

        printf("Saving normal points in %s\n", normal_cloud_save_dir.c_str());
        printf("Saving keyframe poses in %s\n", keyframe_pose_save_path.c_str());
        printf("Saving all poses in %s\n", pose_save_path.c_str());
        printf("Saving lidar poses in %s\n", lidar_pose_save_path.c_str());
        printf("Saving point cloud data in %s\n", cloud_save_dir.c_str());
        printf("Saving normal point cloud data in %s\n", normal_cloud_save_dir.c_str());

        
        FILE * keyframe_pose_file = fopen(keyframe_pose_save_path.c_str(), "w");
        FILE * lidar_pose_file = fopen(lidar_pose_save_path.c_str(), "w");
        FILE * keytonode_file = fopen(keyframe_no_to_node_no_path.c_str(), "w");

        for(int i = 0; i < PointNodes.size(); ++i) {
            if(!PointNodes[i].is_stable) {
                continue;
            }
            char pose[10000];
            sprintf(pose, "%06d\t%.09f\t%.09f\t%.09f\t%.09f\t%.09f\t%.09f\t%.09f\t%.09f\n",
                    i,
                    PointNodes[i].time,
                    PointNodes[i].pose.x(),
                    PointNodes[i].pose.y(),
                    PointNodes[i].pose.z(),
                    PointNodes[i].pose.rotation().toQuaternion().x(),
                    PointNodes[i].pose.rotation().toQuaternion().y(),
                    PointNodes[i].pose.rotation().toQuaternion().z(),
                    PointNodes[i].pose.rotation().toQuaternion().w());
            fwrite(pose, 1, strlen(pose), keyframe_pose_file);

            char lidar_pose[10000];
            sprintf(lidar_pose, "%06d\t%.09f\t%.09f\t%.09f\t%.09f\t%.09f\t%.09f\t%.09f\n",
                    i,
                    PointNodes[i].lidar_pose.x(),
                    PointNodes[i].lidar_pose.y(),
                    PointNodes[i].lidar_pose.z(),
                    PointNodes[i].lidar_pose.rotation().toQuaternion().x(),
                    PointNodes[i].lidar_pose.rotation().toQuaternion().y(),
                    PointNodes[i].lidar_pose.rotation().toQuaternion().z(),
                    PointNodes[i].lidar_pose.rotation().toQuaternion().w());
            fwrite(lidar_pose, 1, strlen(lidar_pose), lidar_pose_file);


            OnSaveNormalPointCloud(i, PointNodes[i]);
            OnSavePointCloud(i, PointNodes[i]);

            char keytonode_data[256];
            sprintf(keytonode_data, "%d\t%d\n", i, PointNodes[i].node_no);
            fwrite(keytonode_data, 1, strlen(keytonode_data), keytonode_file);
        }
        fclose(keyframe_pose_file);
        fclose(lidar_pose_file);
        fclose(keytonode_file);

        FILE * pose_file = fopen(pose_save_path.c_str(), "w");

        char pose_header[10000];
        sprintf(pose_header, "# timestamp_s tx ty tz qx qy qz qw\n");
        fwrite(pose_header, 1, strlen(pose_header), pose_file);
        
        for(int i = 0; i < nodeTimePairs.size(); ++i) {
            gtsam::Pose3 curr_pose = result.at<gtsam::Pose3>(X(nodeTimePairs[i].second));
            double time = nodeTimePairs[i].first;

            char pose[10000];
            if(i == nodeTimePairs.size()-1) {
                sprintf(pose, "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
                                    time,
                                    curr_pose.x(),
                                    curr_pose.y(),
                                    curr_pose.z(),
                                    curr_pose.rotation().toQuaternion().x(),
                                    curr_pose.rotation().toQuaternion().y(),
                                    curr_pose.rotation().toQuaternion().z(),
                                    curr_pose.rotation().toQuaternion().w());
            } else {
                sprintf(pose, "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n",
                                    time,
                                    curr_pose.x(),
                                    curr_pose.y(),
                                    curr_pose.z(),
                                    curr_pose.rotation().toQuaternion().x(),
                                    curr_pose.rotation().toQuaternion().y(),
                                    curr_pose.rotation().toQuaternion().z(),
                                    curr_pose.rotation().toQuaternion().w());
            }

            fwrite(pose, 1, strlen(pose), pose_file);
        }
        fclose(pose_file);

        FILE * loop_file = fopen(loop_index_save_path.c_str(), "w");

        for(int i = 0; i < loopEdges.size(); ++i) {
            int key_pre = loopEdges[i].first;
            int key_cur = loopEdges[i].second;

            gtsam::Pose3 prev_pose = result.at<gtsam::Pose3>(X(key_pre));
            gtsam::Pose3 curr_pose = result.at<gtsam::Pose3>(X(key_cur));

            char loop_corr[100];
            if(i == loopEdges.size()-1) {
                sprintf(loop_corr, "%.9f %.9f %.9f %.9f %.9f %.9f",
                                    prev_pose.translation().x(),
                                    prev_pose.translation().y(),
                                    prev_pose.translation().z(),
                                    curr_pose.translation().x(),
                                    curr_pose.translation().y(),
                                    curr_pose.translation().z());
            } else {
                sprintf(loop_corr, "%.9f %.9f %.9f %.9f %.9f %.9f\n",
                                    prev_pose.translation().x(),
                                    prev_pose.translation().y(),
                                    prev_pose.translation().z(),
                                    curr_pose.translation().x(),
                                    curr_pose.translation().y(),
                                    curr_pose.translation().z());
            }

            fwrite(loop_corr, 1, strlen(loop_corr), loop_file);
        }
        fclose(loop_file);

        //Save Graph
        printf("Saving Graph...\n");
        if(GTSAM_VERSION_MAJOR == 4 && GTSAM_VERSION_MINOR >= 2) {
            gtsam::NonlinearFactorGraph save_graph;
            for(int i = 0; i < vec_priorPoseFactors.size(); ++i) {
                save_graph.add(vec_priorPoseFactors[i]);
            }
            for(int i = 0; i < vec_priorVelFactors.size(); ++i) {
                save_graph.add(vec_priorVelFactors[i]);
            }
            for(int i = 0; i < vec_priorBiasFactors.size(); ++i) {
                save_graph.add(vec_priorBiasFactors[i]);
            }
            for(int i = 0; i < vec_betweenFactorsPoses.size(); ++i) {
                save_graph.add(vec_betweenFactorsPoses[i]);
            }
            for(int i = 0; i < vec_betweenFactorsBias.size(); ++i) {
                save_graph.add(vec_betweenFactorsBias[i]);
            }
            for(int i = 0; i < vec_imuFactors.size(); ++i) {
                save_graph.add(vec_imuFactors[i]);
            }

            gtsam::ISAM2Params optParameters;
            optParameters.relinearizeThreshold = 0.1;
            optParameters.relinearizeSkip = 1;
            gtsam::ISAM2 optimizer_result = gtsam::ISAM2(optParameters);

            optimizer_result.update(save_graph, vec_graphValues);
            gtsam::Values result_result;

            result_result = optimizer_result.calculateEstimate();

            //Save Graph
            
            gtsam::serializeToFile(save_graph, graph_save_path);
            gtsam::serializeToFile(result_result, graph_values_save_path);

        } else {
            printf("GTSAM VERSION: %d.%d may not support serialization\n\033[1;31mGraph Not Saved\033[0m\n", 
                    static_cast<int>(GTSAM_VERSION_MAJOR), static_cast<int>(GTSAM_VERSION_MINOR));
        }
    }

    printf("\033[1;34mSaving Finished!\n\033[0m");

}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "nv_lidar_inertial_mapping_node");
    ros::NodeHandle nh;

    std::vector<double> initialPoseVector;
    std::vector<double> extrinsicRotVector;
    std::vector<double> extrinsicTransVector;

    std::string imuTopic;
    nh.param<std::string>("nv_liom/imuTopic", imuTopic, "/gx5/imu/data");
    nh.param<float>("nv_liom/mapping_voxel_size", voxel_size, 0.2);
    nh.param<float>("nv_liom/visualization_voxel_size", vis_voxel_size, 0.3);
    nh.param<float>("nv_liom/save_voxel_size", save_voxel_size, 0.2);
    nh.param<float>("nv_liom/mapping_range", mapping_range, 1.0);
    nh.param<float>("nv_liom/mapping_closest_range", mapping_closest_range, 0.5);
    nh.param<float>("nv_liom/mapping_node_distance", mapping_node_distance, 0.5);
    nh.param<int>("nv_liom/submap_size", submap_size, 3);
    nh.param<int>("nv_liom/visaulization_node_skip", vis_node_skip, 5);
    nh.param<std::string>("nv_liom/mapping_save_dir", map_save_dir, "/Download/NV_LIOM");
    nh.param<int>("nv_liom/mapping_in_rgb", mapping_in_rgb, 0);
    nh.param<float>("nv_liom/gravity", gravity, 9.8);
    nh.param<std::vector<double>>("nv_liom/lidar_to_imu_R", extrinsicRotVector, std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    nh.param<std::vector<double>>("nv_liom/lidar_to_imu_t", extrinsicTransVector, std::vector<double>{0, 0, 0});
    nh.param<std::vector<double>>("nv_liom/initial_pose", initialPoseVector, std::vector<double>{1, 0 ,0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});

    nh.param<int>("nv_liom/horizontal_pixel_num", hor_pixel_num, 1024);
    nh.param<int>("nv_liom/vertical_pixel_num", ver_pixel_num, 64);
    nh.param<float>("nv_liom/horizontal_fov", hor_fov, 360.0);
    nh.param<float>("nv_liom/vertical_max", ver_max, 22.5);
    nh.param<float>("nv_liom/vertical_min", ver_min, -22.5);
    nh.param<int>("nv_liom/show_img", show_img, 0);

    nh.param<double>("nv_liom/imuAccNoise", imuAccNoise, 3.9939570888238808e-03);
    nh.param<double>("nv_liom/imuGyrNoise", imuGyrNoise, 1.5636343949698187e-03);
    nh.param<double>("nv_liom/imuAccBiasN", imuAccBiasN, 6.4356659353532566e-05);
    nh.param<double>("nv_liom/imuGyrBiasN", imuGyrBiasN, 3.5640318696367613e-05);

    nh.param<double>("nv_liom/correctionNoiseAng", correctionNoiseAng, 0.001);
    nh.param<double>("nv_liom/correctionNoiseTrans", correctionNoiseTrans, 0.01);


    hor_resolution = (hor_fov * M_PI/180.0f)/float(hor_pixel_num);
    ver_resolution = ((ver_max-ver_min) * M_PI/180.0f)/float(ver_pixel_num);


    lidar_to_imu_R = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extrinsicRotVector.data(), 3, 3);
    lidar_to_imu_t = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extrinsicTransVector.data(), 3, 1);
    initial_pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(initialPoseVector.data(), 4, 4);

    lidar_to_imu_T = Eigen::Matrix4d::Identity(4, 4);
    lidar_to_imu_T.block(0, 0, 3, 3) = lidar_to_imu_R;
    lidar_to_imu_T.block(0, 3, 3 ,1) = lidar_to_imu_t;

    imu_to_lidar_T = Eigen::Matrix4d::Identity(4, 4);
    imu_to_lidar_T.block(0, 0, 3, 3) = lidar_to_imu_R.transpose();
    imu_to_lidar_T.block(0, 3, 3, 1) = -lidar_to_imu_R.transpose() * lidar_to_imu_t;

    cloud_save_dir = map_save_dir + "/points";
    normal_cloud_save_dir = map_save_dir + "/normal_points";
    keyframe_pose_save_path = map_save_dir + "/poses.txt";
    pose_save_path = map_save_dir + "/all_poses.txt";
    lidar_pose_save_path = map_save_dir + "/lidar_poses.txt";
    graph_save_path = map_save_dir + "/graph.txt";
    graph_values_save_path = map_save_dir + "/graph_values.txt";
    keyframe_no_to_node_no_path = map_save_dir + "/keyframe_no_to_node_no.txt";
    loop_index_save_path = map_save_dir + "/loop_indices.txt";

    ros::Subscriber subNormalCloud = nh.subscribe<sensor_msgs::PointCloud2>("/nv_liom/normal_vector_cloud", 1, OnSubscribeNormalCloud);
    ros::Subscriber subIMU = nh.subscribe<sensor_msgs::Imu>(imuTopic, 1, OnSubscribeIMU);
    ros::Subscriber subMappingEnd = nh.subscribe<std_msgs::Bool>("nv_liom/end_mapping", 1, OnReceiveEndMappingMessage);


    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/nv_liom/current_cloud", 1);
    pubCloudOptimized = nh.advertise<sensor_msgs::PointCloud2>("/nv_liom/optimized_cloud", 1);
    pubOdometry = nh.advertise<nav_msgs::Odometry>("/nv_liom/odometry", 1);
    pubFactorEdge = nh.advertise<visualization_msgs::MarkerArray>("/nv_liom/factor_edges", 1);
    pubCurrLoopClosure = nh.advertise<visualization_msgs::MarkerArray>("/nv_liom/curr_loops", 1);
    pubLoopClosure = nh.advertise<visualization_msgs::MarkerArray>("/nv_liom/loops", 1);
    pubImuOdometry = nh.advertise<nav_msgs::Odometry>("/nv_liom/imu_odometry", 1);


    pubLoopCurrOdometry = nh.advertise<nav_msgs::Odometry>("/nv_liom/loop_odometry_curr", 1);
    pubLoopTargetOdometry = nh.advertise<nav_msgs::Odometry>("/nv_liom/loop_odometry_target", 1);
    pubLoopCloud_curr = nh.advertise<sensor_msgs::PointCloud2>("/nv_liom/loop_clouds_curr", 1);
    pubLoopCloud_target = nh.advertise<sensor_msgs::PointCloud2>("/nv_liom/loop_clouds_target", 1);

    pubAngularBias = nh.advertise<std_msgs::Float64MultiArray>("/nv_liom/ang_bias", 1);

    cm.SetParams(mapping_closest_range, voxel_size, 
                 hor_fov, ver_max, ver_min, hor_pixel_num, ver_pixel_num, 
                 lidar_to_imu_T, imu_to_lidar_T,
                 show_img);

    voxelGrid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxelGridFilterMap.setLeafSize(save_voxel_size, save_voxel_size, save_voxel_size);
    voxelGridFilterVis.setLeafSize(vis_voxel_size, vis_voxel_size, vis_voxel_size);
    voxelGridFilterVis_rgb.setLeafSize(vis_voxel_size, vis_voxel_size, vis_voxel_size);

    key = 0;
    mapPublish = true;

    resetGraph();

    ROS_INFO("\033[1;34mLiDAR Mapping Node Started\033[0m");

    doneFirstOpt = false;

    lastImuOptTime = -1.0;
    lastImuMsgTime = -1.0;

    publishMapThread = std::thread(PublishMapThread);

    loop_closed_history = 0;
    min_gap = (mapping_range / mapping_node_distance) - 5;

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    
    return 0;
}