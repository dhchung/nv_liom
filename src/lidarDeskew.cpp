#include "utils.h"

std::mutex mtx;

std::deque<sensor_msgs::Imu> imuQueue;

int poseSize;
std::vector<double> imuTime;
std::vector<Eigen::Matrix3d> imuRotMat;

ros::Publisher pubLiDARDeskew;
ros::Publisher pubLiDAR;

pcl::PointCloud<OusterPoint>::Ptr cloud_now;
pcl::PointCloud<HesaiPoint>::Ptr hesai_cloud_now;
pcl::PointCloud<VelodynePointXYZIRT>::Ptr velodyne_cloud_now;
pcl::PointCloud<LivoxPoint>::Ptr livox_cloud_now;

double ang_bias_x;
double ang_bias_y;
double ang_bias_z;

bool do_deskew;
FILE * time_file;

std::string map_save_dir;

Eigen::Matrix3d lidar_to_imu_R;
Eigen::Vector3d lidar_to_imu_t;


void OnSubscribeIMU(const sensor_msgs::ImuConstPtr & msg);
void OnSubscribeLiDARPointCloud_Hesai(const sensor_msgs::PointCloud2ConstPtr & msg);
void OnSubscribeVelodynePointCloud(const sensor_msgs::PointCloud2ConstPtr & msg);
void OnSubscribeLiDARPointCloud(const sensor_msgs::PointCloud2ConstPtr & msg);
bool EstimateRotation();
sensor_msgs::Imu imuTransformation(const sensor_msgs::Imu & imu);
Eigen::Matrix3d rotInterpolation(const Eigen::Matrix3d mat1, const Eigen::Matrix3d mat2, double ratio);

Eigen::Matrix3d rotInterpolation(const Eigen::Matrix3d mat1, const Eigen::Matrix3d mat2, double ratio) {
    Eigen::Matrix3d rel_rot = mat1.transpose() * mat2;
    Eigen::AngleAxisd angAxis(rel_rot);
    Eigen::Vector3d axis = angAxis.axis();
    double angle = angAxis.angle();

    angle *= ratio;
    rel_rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

    return mat1*rel_rot;
}


sensor_msgs::Imu imuTransformation(const sensor_msgs::Imu & imu) {
    sensor_msgs::Imu imu_out = imu;
    Eigen::Vector3d acc_lidar = lidar_to_imu_R * Eigen::Vector3d(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    Eigen::Vector3d angular_vel = lidar_to_imu_R * Eigen::Vector3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
    imu_out.angular_velocity.x = angular_vel.x();
    imu_out.angular_velocity.y = angular_vel.y();
    imu_out.angular_velocity.z = angular_vel.z();
    imu_out.linear_acceleration.x = acc_lidar.x();
    imu_out.linear_acceleration.y = acc_lidar.y();
    imu_out.linear_acceleration.z = acc_lidar.z();
    return imu_out;
}


bool EstimateRotation() {
    mtx.lock();

    imuTime = std::vector<double>(imuQueue.size(), 0.0);
    imuRotMat = std::vector<Eigen::Matrix3d>(imuQueue.size(), Eigen::Matrix3d::Identity());
    poseSize = 0;
    for(size_t i = 0; i < imuQueue.size(); ++i) {

        sensor_msgs::Imu imu_t = imuTransformation(imuQueue[i]);
        if(i == 0) {
            imuTime[i] = imu_t.header.stamp.toSec();
            imuRotMat[i] = Eigen::Matrix3d::Identity(3, 3);
            ++poseSize;
            continue;
        }
        imuTime[i] = imu_t.header.stamp.toSec();
        double dt = imuTime[i] - imuTime[i-1];

        double angular_vel_x = imu_t.angular_velocity.x - ang_bias_x;
        double angular_vel_y = imu_t.angular_velocity.y - ang_bias_y;
        double angular_vel_z = imu_t.angular_velocity.z - ang_bias_z;

        Eigen::Matrix3d skew_R = Eigen::Matrix3d::Zero(3, 3);
        skew_R(0, 1) = -angular_vel_z;
        skew_R(0, 2) = angular_vel_y;
        skew_R(1, 0) = angular_vel_z;
        skew_R(1, 2) = -angular_vel_x;
        skew_R(2, 0) = -angular_vel_y;
        skew_R(2, 1) = angular_vel_x;

        imuRotMat[i] = imuRotMat[i-1] * (Eigen::Matrix3d::Identity(3, 3) + skew_R * dt);
        //normalize
        imuRotMat[i] = Eigen::AngleAxisd(imuRotMat[i]).toRotationMatrix();
        ++poseSize;
    }
    mtx.unlock();

    if(poseSize == 0) {
        return false;
    }

    return true;
}

void OnSubscribeIMU(const sensor_msgs::ImuConstPtr & msg) {
    mtx.lock();
    imuQueue.push_back(*msg);

    mtx.unlock();
}

void OnSubscribeAngBias(const std_msgs::Float64MultiArrayConstPtr & msg) {
    ang_bias_x = msg->data[0];
    ang_bias_y = msg->data[1];
    ang_bias_z = msg->data[2];
}

void OnSubscribeLiDARPointCloud_Hesai(const sensor_msgs::PointCloud2ConstPtr & msg) {
    pcl::fromROSMsg(*msg, *hesai_cloud_now);
    if(hesai_cloud_now->empty()) {
        return;
    }

    sensor_msgs::PointCloud2 transformed_msg;
    pcl::toROSMsg(*hesai_cloud_now, transformed_msg);
    transformed_msg.header = msg->header;
    transformed_msg.header.frame_id = "lidar";

    std::vector<double> point_times;
    for(size_t i = 0; i < hesai_cloud_now->size(); ++i) {
        point_times.push_back(hesai_cloud_now->points[i].timestamp);
    }
    std::sort(point_times.begin(), point_times.end());
    double min_time = point_times.front();
    double max_time = point_times.back();

    if(max_time - min_time > 0.15) {

        printf("wrong points\n");
        return;
    }

    if(!do_deskew) {
        pubLiDARDeskew.publish(transformed_msg);
        pubLiDAR.publish(transformed_msg);
        return;
    }

    double imu_front_time = 0.0;
    double imu_back_time = 0.0;

    mtx.lock();
    if(imuQueue.empty()) {
        mtx.unlock();
        return;
    }

    imu_front_time = imuQueue.front().header.stamp.toSec();
    imu_back_time = imuQueue.back().header.stamp.toSec();
    if(imu_front_time > min_time) {
        // IMU minimum time is larger than the point time: invalid point cloud
        mtx.unlock();
        return;
    }
    sensor_msgs::Imu imu_front;
    while(imuQueue.front().header.stamp.toSec() < min_time) {
        imu_front = imuQueue.front();
        imuQueue.pop_front();
        if(imuQueue.empty()) {
            mtx.unlock();
            return;
        }
    }
    imuQueue.push_front(imu_front);

    mtx.unlock();

    while(imu_back_time < max_time) {
        // Wait untill imuQueue fills up to cover all point times
        usleep(1000);
        mtx.lock();
        imu_back_time = imuQueue.back().header.stamp.toSec();
        mtx.unlock();
    }


    if(!EstimateRotation()) {
        pubLiDARDeskew.publish(transformed_msg);
        pubLiDAR.publish(transformed_msg);
        return;
    }

    //Estimate the first rotation
    int first_imu_index = 0;
    while(imuTime[first_imu_index] < min_time) {
        ++first_imu_index;
        if(first_imu_index >= poseSize) {
            break;
        }
    }

    double first_rotX = 0.0;
    double first_rotY = 0.0;
    double first_rotZ = 0.0;
    Eigen::Matrix3d first_rotMat = Eigen::Matrix3d::Identity();

    if(first_imu_index == 0 || imuTime[first_imu_index] < min_time) {
        first_rotMat = imuRotMat[first_imu_index];
    } else {
        double ratio = (min_time - imuTime[first_imu_index -1])/(imuTime[first_imu_index] - imuTime[first_imu_index - 1]);
        first_rotMat = rotInterpolation(imuRotMat[first_imu_index-1], imuRotMat[first_imu_index], ratio);
    }


    #pragma omp parallel for num_threads(4)
    for(size_t i = 0; i < hesai_cloud_now->size(); ++i) {
        HesaiPoint & pt = hesai_cloud_now->points[i];
        int imu_index = 0;

        double point_time = pt.timestamp;

        double rotX;
        double rotY;
        double rotZ;

        Eigen::Matrix3d rotMat;

        while(imuTime[imu_index] < point_time) {
            ++imu_index;
            if(imu_index >= poseSize) {
                printf("imu index exceeds est_pose count: %d, %d\n", imu_index, poseSize);
                printf("imu time: %f[s], point time: %f[s]\n", imuTime[imu_index], point_time);
                break;
            }
        }

        if(imu_index == 0 || imuTime[imu_index] < point_time) {
            rotMat = first_rotMat.transpose() * imuRotMat[imu_index];
        }  else {

            double ratio = (point_time - imuTime[imu_index -1])/(imuTime[imu_index] - imuTime[imu_index - 1]);
            rotMat = first_rotMat.transpose() * (rotInterpolation(imuRotMat[imu_index-1], imuRotMat[imu_index], ratio));
        }


        Eigen::Vector3d pt_vec = rotMat * Eigen::Vector3d(pt.x, pt.y, pt.z);
        pt.x = pt_vec.x();
        pt.y = pt_vec.y();
        pt.z = pt_vec.z();

    }


    sensor_msgs::PointCloud2 deskew_msg;
    pcl::toROSMsg(*hesai_cloud_now, deskew_msg);
    deskew_msg.header = msg->header;
    deskew_msg.header.frame_id = "lidar";
    pubLiDARDeskew.publish(deskew_msg);
    pubLiDAR.publish(transformed_msg);

}

void OnSubscribeLiDARPointCloud(const sensor_msgs::PointCloud2ConstPtr & msg) {
    pcl::fromROSMsg(*msg, *cloud_now);
    if(cloud_now->empty()) {
        return;
    }


    sensor_msgs::PointCloud2 transformed_msg;
    pcl::toROSMsg(*cloud_now, transformed_msg);
    transformed_msg.header = msg->header;
    transformed_msg.header.frame_id = "lidar";
    double cloud_time = msg->header.stamp.toSec();

    std::vector<double> point_times;
    for(size_t i = 0; i < cloud_now->size(); ++i) {
        point_times.push_back(static_cast<double>(cloud_now->points[i].t)*1e-9 + cloud_time);
    }
    std::sort(point_times.begin(), point_times.end());
    double min_time = point_times.front();
    double max_time = point_times.back();

    if(max_time - min_time > 0.15) {
        printf("wrong points\n");
        return;
    }

    if(!do_deskew) {
        pubLiDARDeskew.publish(transformed_msg);
        pubLiDAR.publish(transformed_msg);
        return;
    }

    double imu_front_time = 0.0;
    double imu_back_time = 0.0;
    mtx.lock();

    if(imuQueue.empty()) {
        mtx.unlock();
        return;
    }


    imu_front_time = imuQueue.front().header.stamp.toSec();
    imu_back_time = imuQueue.back().header.stamp.toSec();
    if(imu_front_time > min_time) {
        // IMU minimum time is larger than the point time: invalid point cloud
        mtx.unlock();
        return;
    }
    sensor_msgs::Imu imu_front;
    while(imuQueue.front().header.stamp.toSec() < min_time) {
        imu_front = imuQueue.front();
        imuQueue.pop_front();
        if(imuQueue.empty()) {
            mtx.unlock();
            return;
        }
    }
    imuQueue.push_front(imu_front);

    mtx.unlock();

    while(imu_back_time < max_time) {
        // Wait untill imuQueue fills up to cover all point times
        mtx.lock();
        imu_back_time = imuQueue.back().header.stamp.toSec();
        mtx.unlock();
    }


    if(!EstimateRotation()) {
        pubLiDARDeskew.publish(transformed_msg);
        pubLiDAR.publish(transformed_msg);
        return;
    }


    //Estimate the first rotation
    int first_imu_index = 0;
    while(imuTime[first_imu_index] < min_time) {
        ++first_imu_index;
        if(first_imu_index >= poseSize) {
            break;
        }
    }

    double first_rotX = 0.0;
    double first_rotY = 0.0;
    double first_rotZ = 0.0;
    Eigen::Matrix3d first_rotMat = Eigen::Matrix3d::Identity();

    if(first_imu_index == 0 || imuTime[first_imu_index] < min_time) {
        first_rotMat = imuRotMat[first_imu_index];
    } else {
        double ratio = (min_time - imuTime[first_imu_index -1])/(imuTime[first_imu_index] - imuTime[first_imu_index - 1]);
        first_rotMat = rotInterpolation(imuRotMat[first_imu_index-1], imuRotMat[first_imu_index], ratio);
    }


    #pragma omp parallel for num_threads(4)
    for(size_t i = 0; i < cloud_now->size(); ++i) {
        OusterPoint & pt = cloud_now->points[i];
        int imu_index = 0;

        double point_time = static_cast<double>(pt.t)*1e-9 + cloud_time;

        double rotX;
        double rotY;
        double rotZ;

        Eigen::Matrix3d rotMat;

        while(imuTime[imu_index] < point_time) {
            ++imu_index;
            if(imu_index >= poseSize) {
                printf("imu index exceeds est_pose count: %d, %d\n", imu_index, poseSize);
                printf("imu time: %f[s], point time: %f[s]\n", imuTime[imu_index], point_time);
                break;
            }
        }

        if(imu_index == 0 || imuTime[imu_index] < point_time) {
            rotMat = first_rotMat.transpose() * imuRotMat[imu_index];
        }  else {

            double ratio = (point_time - imuTime[imu_index -1])/(imuTime[imu_index] - imuTime[imu_index - 1]);
            rotMat = first_rotMat.transpose() * (rotInterpolation(imuRotMat[imu_index-1], imuRotMat[imu_index], ratio));
        }


        Eigen::Vector3d pt_vec = rotMat * Eigen::Vector3d(pt.x, pt.y, pt.z);
        pt.x = pt_vec.x();
        pt.y = pt_vec.y();
        pt.z = pt_vec.z();

    }


    sensor_msgs::PointCloud2 deskew_msg;
    pcl::toROSMsg(*cloud_now, deskew_msg);
    deskew_msg.header = msg->header;
    deskew_msg.header.frame_id = "lidar";
    pubLiDARDeskew.publish(deskew_msg);
    pubLiDAR.publish(transformed_msg);

}


void OnSubscribeVelodynePointCloud(const sensor_msgs::PointCloud2ConstPtr & msg) {
    pcl::fromROSMsg(*msg, *velodyne_cloud_now);
    if(velodyne_cloud_now->empty()) {
        return;
    }

    sensor_msgs::PointCloud2 transformed_msg;
    pcl::toROSMsg(*velodyne_cloud_now, transformed_msg);
    transformed_msg.header = msg->header;
    transformed_msg.header.frame_id = "lidar";
    double cloud_time = msg->header.stamp.toSec();

    std::vector<double> point_times;
    for(size_t i = 0; i < velodyne_cloud_now->size(); ++i) {
        point_times.push_back(static_cast<double>(velodyne_cloud_now->points[i].time) + cloud_time);
    }
    std::sort(point_times.begin(), point_times.end());
    double min_time = point_times.front();
    double max_time = point_times.back();

    if(max_time - min_time > 0.15) {
        printf("wrong points\n");
        return;
    }

    if(!do_deskew) {
        pubLiDARDeskew.publish(transformed_msg);
        pubLiDAR.publish(transformed_msg);
        return;
    }

    double imu_front_time = 0.0;
    double imu_back_time = 0.0;
    mtx.lock();

    if(imuQueue.empty()) {
        mtx.unlock();
        return;
    }


    imu_front_time = imuQueue.front().header.stamp.toSec();
    imu_back_time = imuQueue.back().header.stamp.toSec();
    if(imu_front_time > min_time) {
        // IMU minimum time is larger than the point time: invalid point cloud
        mtx.unlock();
        return;
    }
    sensor_msgs::Imu imu_front;
    while(imuQueue.front().header.stamp.toSec() < min_time) {
        imu_front = imuQueue.front();
        imuQueue.pop_front();
        if(imuQueue.empty()) {
            mtx.unlock();
            return;
        }
    }
    imuQueue.push_front(imu_front);

    mtx.unlock();

    while(imu_back_time < max_time) {
        // Wait untill imuQueue fills up to cover all point times
        mtx.lock();
        imu_back_time = imuQueue.back().header.stamp.toSec();
        mtx.unlock();
    }


    if(!EstimateRotation()) {
        pubLiDARDeskew.publish(transformed_msg);
        pubLiDAR.publish(transformed_msg);
        return;
    }


    //Estimate the first rotation
    int first_imu_index = 0;
    while(imuTime[first_imu_index] < min_time) {
        ++first_imu_index;
        if(first_imu_index >= poseSize) {
            break;
        }
    }

    double first_rotX = 0.0;
    double first_rotY = 0.0;
    double first_rotZ = 0.0;
    Eigen::Matrix3d first_rotMat = Eigen::Matrix3d::Identity();

    if(first_imu_index == 0 || imuTime[first_imu_index] < min_time) {
        first_rotMat = imuRotMat[first_imu_index];
    } else {
        double ratio = (min_time - imuTime[first_imu_index -1])/(imuTime[first_imu_index] - imuTime[first_imu_index - 1]);
        first_rotMat = rotInterpolation(imuRotMat[first_imu_index-1], imuRotMat[first_imu_index], ratio);
    }


    #pragma omp parallel for num_threads(4)
    for(size_t i = 0; i < velodyne_cloud_now->size(); ++i) {
        VelodynePointXYZIRT & pt = velodyne_cloud_now->points[i];
        int imu_index = 0;

        double point_time = static_cast<double>(pt.time) + cloud_time;

        double rotX;
        double rotY;
        double rotZ;

        Eigen::Matrix3d rotMat;

        while(imuTime[imu_index] < point_time) {
            ++imu_index;
            if(imu_index >= poseSize) {
                printf("imu index exceeds est_pose count: %d, %d\n", imu_index, poseSize);
                printf("imu time: %f[s], point time: %f[s]\n", imuTime[imu_index], point_time);
                break;
            }
        }

        if(imu_index == 0 || imuTime[imu_index] < point_time) {
            rotMat = first_rotMat.transpose() * imuRotMat[imu_index];
        }  else {

            double ratio = (point_time - imuTime[imu_index -1])/(imuTime[imu_index] - imuTime[imu_index - 1]);
            rotMat = first_rotMat.transpose() * (rotInterpolation(imuRotMat[imu_index-1], imuRotMat[imu_index], ratio));
        }


        Eigen::Vector3d pt_vec = rotMat * Eigen::Vector3d(pt.x, pt.y, pt.z);
        pt.x = pt_vec.x();
        pt.y = pt_vec.y();
        pt.z = pt_vec.z();

    }


    sensor_msgs::PointCloud2 deskew_msg;
    pcl::toROSMsg(*velodyne_cloud_now, deskew_msg);
    deskew_msg.header = msg->header;
    deskew_msg.header.frame_id = "lidar";
    pubLiDARDeskew.publish(deskew_msg);
    pubLiDAR.publish(transformed_msg);


}

void OnSubscribeLiDARPointCloud_Livox(const sensor_msgs::PointCloud2ConstPtr & msg) {
    pcl::fromROSMsg(*msg, *livox_cloud_now);
    if(livox_cloud_now->empty()) {
        printf("Livox Cloud Empty\n");
        return;
    }

    sensor_msgs::PointCloud2 transformed_msg;
    pcl::toROSMsg(*livox_cloud_now, transformed_msg);
    transformed_msg.header = msg->header;
    transformed_msg.header.frame_id = "lidar";
    double cloud_time = msg->header.stamp.toSec();

    std::vector<double> point_times;
    for(size_t i = 0; i < livox_cloud_now->size(); ++i) {
        point_times.push_back(static_cast<double>(livox_cloud_now->points[i].timestamp)*1e-9);
    }
    std::sort(point_times.begin(), point_times.end());
    double min_time = point_times.front();
    double max_time = point_times.back();

    if(max_time - min_time > 0.15) {

        printf("wrong points: %f\n", max_time - min_time);
        return;
    }

    if(!do_deskew) {
        printf("No Deskew\n");

        pubLiDARDeskew.publish(transformed_msg);
        pubLiDAR.publish(transformed_msg);
        return;
    }

    double imu_front_time = 0.0;
    double imu_back_time = 0.0;

    mtx.lock();
    if(imuQueue.empty()) {
        printf("IMU Empty\n");

        mtx.unlock();
        return;
    }

    imu_front_time = imuQueue.front().header.stamp.toSec();
    imu_back_time = imuQueue.back().header.stamp.toSec();
    if(imu_front_time > min_time) {
        printf("IMU Invalid\n");

        // IMU minimum time is larger than the point time: invalid point cloud
        mtx.unlock();
        return;
    }
    sensor_msgs::Imu imu_front;
    while(imuQueue.front().header.stamp.toSec() < min_time) {
        printf("imuFront Time: %f\n", imuQueue.front().header.stamp.toSec());

        imu_front = imuQueue.front();
        imuQueue.pop_front();
        if(imuQueue.empty()) {
            printf("IMU Empty2\n");
            printf("min Time: %f\n", min_time);

            mtx.unlock();
            return;
        }
    }
    imuQueue.push_front(imu_front);

    mtx.unlock();

    while(imu_back_time < max_time) {
        // Wait untill imuQueue fills up to cover all point times
        usleep(1000);
        mtx.lock();
        imu_back_time = imuQueue.back().header.stamp.toSec();
        mtx.unlock();
    }


    if(!EstimateRotation()) {
        pubLiDARDeskew.publish(transformed_msg);
        pubLiDAR.publish(transformed_msg);
        return;
    }

    //Estimate the first rotation
    int first_imu_index = 0;
    while(imuTime[first_imu_index] < min_time) {
        ++first_imu_index;
        if(first_imu_index >= poseSize) {
            break;
        }
    }

    double first_rotX = 0.0;
    double first_rotY = 0.0;
    double first_rotZ = 0.0;
    Eigen::Matrix3d first_rotMat = Eigen::Matrix3d::Identity();

    if(first_imu_index == 0 || imuTime[first_imu_index] < min_time) {
        first_rotMat = imuRotMat[first_imu_index];
    } else {
        double ratio = (min_time - imuTime[first_imu_index -1])/(imuTime[first_imu_index] - imuTime[first_imu_index - 1]);
        first_rotMat = rotInterpolation(imuRotMat[first_imu_index-1], imuRotMat[first_imu_index], ratio);
    }


    #pragma omp parallel for num_threads(4)
    for(size_t i = 0; i < hesai_cloud_now->size(); ++i) {
        HesaiPoint & pt = hesai_cloud_now->points[i];
        int imu_index = 0;

        double point_time = pt.timestamp;

        double rotX;
        double rotY;
        double rotZ;

        Eigen::Matrix3d rotMat;

        while(imuTime[imu_index] < point_time) {
            ++imu_index;
            if(imu_index >= poseSize) {
                printf("imu index exceeds est_pose count: %d, %d\n", imu_index, poseSize);
                printf("imu time: %f[s], point time: %f[s]\n", imuTime[imu_index], point_time);
                break;
            }
        }

        if(imu_index == 0 || imuTime[imu_index] < point_time) {
            rotMat = first_rotMat.transpose() * imuRotMat[imu_index];
        }  else {

            double ratio = (point_time - imuTime[imu_index -1])/(imuTime[imu_index] - imuTime[imu_index - 1]);
            rotMat = first_rotMat.transpose() * (rotInterpolation(imuRotMat[imu_index-1], imuRotMat[imu_index], ratio));
        }


        Eigen::Vector3d pt_vec = rotMat * Eigen::Vector3d(pt.x, pt.y, pt.z);
        pt.x = pt_vec.x();
        pt.y = pt_vec.y();
        pt.z = pt_vec.z();

    }


    sensor_msgs::PointCloud2 deskew_msg;
    pcl::toROSMsg(*hesai_cloud_now, deskew_msg);
    deskew_msg.header = msg->header;
    deskew_msg.header.frame_id = "lidar";
    pubLiDARDeskew.publish(deskew_msg);
    pubLiDAR.publish(transformed_msg);


}


int main(int argc, char ** argv) {

    ros::init(argc, argv, "nv_lidar_deskew_node");
    ros::NodeHandle nh;

    std::string lidarTopic;
    std::string imuTopic;
    int deskew_flag;
    int lidar_type;

    std::vector<double> extrinsicRotVector;
    std::vector<double> extrinsicTransVector;

    ang_bias_x = 0.0;
    ang_bias_y = 0.0;
    ang_bias_z = 0.0;

    nh.param<int>("nv_liom/lidar_type", lidar_type, 0);
    nh.param<std::string>("nv_liom/imuTopic", imuTopic, "/ouster/imu");
    nh.param<std::string>("nv_liom/lidarTopic", lidarTopic, "/ouster/points");
    nh.param<int>("nv_liom/deskew", deskew_flag, 0);
    nh.param<std::string>("nv_liom/mapping_save_dir", map_save_dir, "/home/morin/map");
    nh.param<std::vector<double>>("nv_liom/lidar_to_imu_R", extrinsicRotVector, std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    nh.param<std::vector<double>>("nv_liom/lidar_to_imu_t", extrinsicTransVector, std::vector<double>{0, 0, 0});

    lidar_to_imu_R = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extrinsicRotVector.data(), 3, 3);
    lidar_to_imu_t = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extrinsicTransVector.data(), 3, 1);
    poseSize = 0;

    do_deskew = deskew_flag == 1 ? true : false;

    ros::Subscriber subLiDARPointCloud;
    if(lidar_type == 0) {
        subLiDARPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 1, OnSubscribeLiDARPointCloud);
    } else if (lidar_type == 1) {
        subLiDARPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 1, OnSubscribeLiDARPointCloud_Hesai);
    } else if (lidar_type == 2) {
        subLiDARPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 1, OnSubscribeVelodynePointCloud);
    }
    
    ros::Subscriber subIMU = nh.subscribe<sensor_msgs::Imu>(imuTopic, 20000, OnSubscribeIMU);
    ros::Subscriber subBias = nh.subscribe<std_msgs::Float64MultiArray>("/nv_liom/ang_bias", 2000, OnSubscribeAngBias);

    pubLiDARDeskew = nh.advertise<sensor_msgs::PointCloud2>("/nv_liom/deskew_cloud", 1000);
    pubLiDAR = nh.advertise<sensor_msgs::PointCloud2>("/nv_liom/original_cloud", 1000);

    cloud_now.reset(new pcl::PointCloud<OusterPoint>());
    hesai_cloud_now.reset(new pcl::PointCloud<HesaiPoint>());
    velodyne_cloud_now.reset(new pcl::PointCloud<VelodynePointXYZIRT>());

    ROS_INFO("\033[1;34mLiDAR Deskew Node Started\033[0m");
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    
    return 0;
}