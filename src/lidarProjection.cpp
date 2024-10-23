#include "utils.h"

std::mutex mtx;
std::deque<sensor_msgs::Imu> imuQueue;

pcl::PointCloud<pcl::PointXYZI>::Ptr deskew_cloud;
pcl::PointCloud<ProjectedPoint>::Ptr projected_cloud;

ros::Publisher pubLiDARProjection;
FILE * time_file;

std::string map_save_dir;

int hor_pixel_num;
int ver_pixel_num;
float hor_fov;
float ver_max;
float ver_min;

float min_dist;
float max_dist;

float hor_resolution;
float ver_resolution;
int show_img;

int min_intensity;

template <typename PointType>
void get_uv(PointType point, int & u, int & v) {

    float xy_range = std::sqrt(point.x * point.x + point.y * point.y);
    if(xy_range < 1e-3) {
        u = -1;
        v = -1;
        return;
    }

    // float azimuth = atan2(point.y, point.x) + (hor_fov * M_PI/180.0f / 2.0f);
    // float elevation = atan2(point.z, xy_range) + (ver_fov * M_PI/180.0f / 2.0f);

    float azimuth =  M_PI - atan2(point.y, point.x);
    float elevation = atan2(point.z, xy_range);

    int u_ = azimuth / hor_resolution;
    int v_ = ((ver_max * M_PI/180.0) - elevation) / ver_resolution;

    if(u_ < 0 || u_ >= hor_pixel_num || v_ < 0 || v_ >= ver_pixel_num) {
        u = -1;
        v = -1;
        return;
    }

    u = u_;
    v = v_;
    return;
}

void OnSubscribeDeskewLiDARPointCloud(const sensor_msgs::PointCloud2ConstPtr & msg) {

    pcl::fromROSMsg(*msg, *deskew_cloud);
    projected_cloud.reset(new pcl::PointCloud<ProjectedPoint>(hor_pixel_num, ver_pixel_num, ProjectedPoint()));


    cv::Mat depth_img(ver_pixel_num, hor_pixel_num, CV_8UC1, cv::Scalar(0));

    #pragma omp parallel for num_threads(4)
    for(int i = 0; i < deskew_cloud->size(); ++i) {
        pcl::PointXYZI & pt = deskew_cloud->points[i];
        int u, v;
        get_uv(pt, u, v);

        if(u == -1) {
            continue;
        }
        
        ProjectedPoint & pt_proj = projected_cloud->points[v * hor_pixel_num + u];

        float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if(range < min_dist || range > max_dist || pt.intensity < min_intensity) {
            continue;
        }
        
        if(pt_proj.valid == 1) {
            if(range < pt_proj.range) {
                pt_proj.x = pt.x;
                pt_proj.y = pt.y;
                pt_proj.z = pt.z;
                pt_proj.intensity = pt.intensity;
                pt_proj.range = range;
                continue;
            } else {
                continue;
            }
        }

        pt_proj.x = pt.x;
        pt_proj.y = pt.y;
        pt_proj.z = pt.z;
        pt_proj.intensity = pt.intensity;
        pt_proj.range = range;
        pt_proj.valid = 1;


    }

    sensor_msgs::PointCloud2 projection_msg;
    pcl::toROSMsg(*projected_cloud, projection_msg);
    projection_msg.header = msg->header;
    pubLiDARProjection.publish(projection_msg);

    if(show_img == 1) {

        float max_range = 0.0f;
        for(int i = 0; i < projected_cloud->size(); ++i) {
            max_range = projected_cloud->points[i].range > max_range ? projected_cloud->points[i].range : max_range;
        }
        
        for(int u = 0; u < hor_pixel_num; ++u) {
            for(int v = 0; v < ver_pixel_num; ++v) {
                float range = projected_cloud->points[v * hor_pixel_num + u].range;
                int pix_val = (range/max_range)*255.0;
                if(pix_val > 255) {
                    pix_val = 255;
                }
                depth_img.at<uint8_t>(v, u) = pix_val;
            }
        }


        cv::imshow("Depth Img", depth_img);
        cv::waitKey(1);
    }

}


int main(int argc, char ** argv) {

    ros::init(argc, argv, "nv_lidar_projection_node");
    ros::NodeHandle nh;

    nh.param<int>("nv_liom/horizontal_pixel_num", hor_pixel_num, 1024);
    nh.param<int>("nv_liom/vertical_pixel_num", ver_pixel_num, 64);
    nh.param<float>("nv_liom/horizontal_fov", hor_fov, 360.0);
    nh.param<float>("nv_liom/vertical_max", ver_max, 22.5);
    nh.param<float>("nv_liom/vertical_min", ver_min, -22.5);

    nh.param<float>("nv_liom/minimum_distance", min_dist, 1.0);
    nh.param<float>("nv_liom/maximum_distance", max_dist, 200.0);
    nh.param<int>("nv_liom/show_img", show_img, 0);
    nh.param<int>("nv_liom/min_intensity", min_intensity, 190);
    
    nh.param<std::string>("nv_liom/mapping_save_dir", map_save_dir, "/home/morin/map");
    
    hor_resolution = (hor_fov * M_PI/180.0f)/float(hor_pixel_num);
    ver_resolution = ((ver_max-ver_min) * M_PI/180.0f)/float(ver_pixel_num);

    ros::Subscriber subDeskewLiDARPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/nv_liom/deskew_cloud", 100, OnSubscribeDeskewLiDARPointCloud);
    pubLiDARProjection = nh.advertise<sensor_msgs::PointCloud2>("/nv_liom/projected_cloud", 1000);

    deskew_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    ROS_INFO("\033[1;34mLiDAR Projection Node Started\033[0m");

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();
    
    return 0;
}