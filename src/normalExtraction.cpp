#include "utils.h"

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
ros::Publisher pubNormalPointCloud;

int normal_neighbor;
// ros::Publisher pubNormalPointCloudRGB;

std::string map_save_dir;

std::vector<Eigen::Matrix3f> sp2cart_map;

cv::Mat normal_img;
cv::Mat normal_img_sp;

void index2uv(int index, int & u, int & v) {
    v = index / hor_pixel_num;
    u = index % hor_pixel_num;
    return;
}

int uv2index(int u, int v) {
    return v*hor_pixel_num + u;
}

void OnInitialization() {
    sp2cart_map = std::vector<Eigen::Matrix3f>(hor_pixel_num * ver_pixel_num, Eigen::Matrix3f::Zero(3,3));

    for(int i = 0; i < hor_pixel_num * ver_pixel_num; ++i) {
        int u, v;
        index2uv(i, u, v);
        float psi = M_PI - float(u) * hor_resolution;
        // float theta = (M_PI - (ver_fov * M_PI/180.0))/2.0 + float(v) * ver_resolution;

        float theta = M_PI/2.0 - (ver_max * M_PI/180.0) + (float(v) * ver_resolution);

        Eigen::Matrix3f & mat = sp2cart_map[i];

        mat(0, 0) = -sin(psi);
        mat(0, 1) = cos(psi)*cos(theta);
        mat(0, 2) = cos(psi)*sin(theta);

        mat(1, 0) = cos(psi);
        mat(1, 1) = sin(psi)*cos(theta);
        mat(1, 2) = sin(psi)*sin(theta);
        
        mat(2, 0) = 0.0;
        mat(2, 1) = -sin(theta);
        mat(2, 2) = cos(theta);
    }
}

void NormalExtraction(const pcl::PointCloud<ProjectedPoint>::Ptr & projected_cloud, 
                      pcl::PointCloud<PointsWithNormals>::Ptr &normal_cloud) {

    if(show_img == 1) {
        normal_img = cv::Mat(ver_pixel_num, hor_pixel_num, CV_8UC3, cv::Scalar(0, 0, 0));
        normal_img_sp = cv::Mat(ver_pixel_num, hor_pixel_num, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    #pragma omp parallel for num_threads(4)
    for(int i = 0; i < projected_cloud->size(); ++i) {
        int u, v;
        index2uv(i, u, v);
        
        ProjectedPoint & curr_pt = projected_cloud->points[i];
        normal_cloud->points[i].x = curr_pt.x;
        normal_cloud->points[i].y = curr_pt.y;
        normal_cloud->points[i].z = curr_pt.z;
        normal_cloud->points[i].intensity = curr_pt.intensity;
        normal_cloud->points[i].range = curr_pt.range;

        if(curr_pt.valid == 0) {
            continue;
        }

        if(u + normal_neighbor >= hor_pixel_num || (u-normal_neighbor < 0) ||
           v + normal_neighbor >= ver_pixel_num || (v-normal_neighbor < 0)) {
            continue;
        }

        double dzdpsi_sum = 0.0;
        double dzdtheta_sum = 0.0;

        int dpsi_sample_no = 0;
        int dtheta_sample_no = 0;

        for(int j = -normal_neighbor; j <= normal_neighbor; ++j) {
            for(int k = -normal_neighbor; k <= normal_neighbor; ++k) {
                int query_i = uv2index(u+j, v+k);
                ProjectedPoint & query_pt = projected_cloud->points[query_i];
                if(query_pt.valid == 0) {
                    continue;
                }
                // if(std::abs(query_pt.range - curr_pt.range) > 1.5) {
                //     continue;
                // }
                //Horizontal
                for(int l = j+1; l <= normal_neighbor; ++l) {
                    int target_i = uv2index(u+l, v+k);
                    ProjectedPoint & target_pt = projected_cloud->points[target_i];
                    if(target_pt.valid == 0) {
                        continue;
                    }
                    // if(std::abs(target_pt.range - query_pt.range) > 1.5) {
                    //     continue;
                    // }

                    dzdpsi_sum += (target_pt.range - query_pt.range)/(float(l-j)*hor_resolution*curr_pt.range);
                    ++dpsi_sample_no;
                }

                for(int l = k+1; l <= normal_neighbor; ++l) {
                    int target_i = uv2index(u+j, v+l);
                    ProjectedPoint & target_pt = projected_cloud->points[target_i];
                    if(target_pt.valid == 0) {
                        continue;
                    }
                    // if(std::abs(target_pt.range - query_pt.range) > 1.5) {
                    //     continue;
                    // }

                    dzdtheta_sum += (target_pt.range - query_pt.range)/(float(l-k)*ver_resolution*curr_pt.range);
                    ++dtheta_sample_no;
                }
            }
        }

        if(dpsi_sample_no < normal_neighbor*2 || dtheta_sample_no < normal_neighbor*2) {
            continue;
        }
        float dzdpsi_mean = dzdpsi_sum/float(dpsi_sample_no);
        float dzdtheta_mean = dzdtheta_sum/float(dtheta_sample_no);

        Eigen::Vector3f normal_sp{dzdpsi_mean, -dzdtheta_mean, 1};
        normal_sp.normalize();

        Eigen::Vector3f ray_dir{curr_pt.x, curr_pt.y, curr_pt.z};
        ray_dir.normalize();

        Eigen::Vector3f normal = sp2cart_map[i]*normal_sp;
        normal.normalize();

        if(normal.dot(ray_dir) > 0) {
            normal = -normal;
        }

        float d = -(normal.x()*curr_pt.x + normal.y()*curr_pt.y + normal.z()*curr_pt.z);

        bool valid_normal = true;;

        int valid_neighbors = 0;

        for(int j = -normal_neighbor; j <= normal_neighbor; ++j) {
            for(int k = -normal_neighbor; k <= normal_neighbor; ++k) {
                int query_i = uv2index(u+j, v+k);
                ProjectedPoint & target_pt = projected_cloud->points[query_i];
                float dist = std::abs(d + normal.x()*target_pt.x + normal.y() * target_pt.y + normal.z() *target_pt.z);

                if(dist < 0.05) {
                    ++valid_neighbors;
                }

            }
        }
        if(valid_neighbors < (2*normal_neighbor+1)*(2*normal_neighbor+1)/3) {
            continue;
        }

        normal_cloud->points[i].valid = 1;
        normal_cloud->points[i].normal_x = normal.x();
        normal_cloud->points[i].normal_y = normal.y();
        normal_cloud->points[i].normal_z = normal.z();

        if(show_img == 1) {
            int r = 0;
            int g = 0;
            int b = 0;
            r = int((normal(0)*0.5+0.5) * 255);
            g = int((normal(1)*0.5+0.5) * 255);
            b = int((normal(2)*0.5+0.5) * 255);
            normal_img.at<cv::Vec3b>(v, u) = cv::Vec3b(b, g, r);

            int r_sp = 0;
            int g_sp = 0;
            int b_sp = 0;
            r_sp = int((normal_sp(0)*0.5+0.5) * 255);
            g_sp = int((normal_sp(1)*0.5+0.5) * 255);
            b_sp = int((normal_sp(2)*0.5+0.5) * 255);


            normal_img_sp.at<cv::Vec3b>(v, u) = cv::Vec3b(b_sp, g_sp, r_sp);
        }


    }
    if(show_img==1) {
        cv::vconcat(normal_img, normal_img_sp, normal_img);
        cv::imshow("Normal Img", normal_img);
        cv::waitKey(1);
    }

}

void OnSubscribeProjectedPointCloud(const sensor_msgs::PointCloud2ConstPtr & msg) {
    pcl::PointCloud<ProjectedPoint>::Ptr projected_cloud(new pcl::PointCloud<ProjectedPoint>);
    pcl::fromROSMsg(*msg, *projected_cloud);
    pcl::PointCloud<PointsWithNormals>::Ptr normal_cloud(new pcl::PointCloud<PointsWithNormals>(hor_pixel_num, ver_pixel_num));
    NormalExtraction(projected_cloud, normal_cloud);



    sensor_msgs::PointCloud2 normal_cloud_msg;
    pcl::toROSMsg(*normal_cloud, normal_cloud_msg);
    normal_cloud_msg.header = msg->header;
    pubNormalPointCloud.publish(normal_cloud_msg);

}


int main(int argc, char ** argv) {

    ros::init(argc, argv, "nv_normal_extraction_node");
    ros::NodeHandle nh;

    nh.param<int>("nv_liom/horizontal_pixel_num", hor_pixel_num, 1024);
    nh.param<int>("nv_liom/vertical_pixel_num", ver_pixel_num, 64);
    nh.param<float>("nv_liom/horizontal_fov", hor_fov, 360.0);
    nh.param<float>("nv_liom/vertical_max", ver_max, 22.5);
    nh.param<float>("nv_liom/vertical_min", ver_min, -22.5);

    nh.param<float>("nv_liom/minimum_distance", min_dist, 1.0);
    nh.param<float>("nv_liom/maximum_distance", max_dist, 200.0);
    nh.param<int>("nv_liom/show_img", show_img, 1);
    nh.param<std::string>("nv_liom/mapping_save_dir", map_save_dir, "/home/morin/map");
    nh.param<int>("nv_liom/normal_neighbor", normal_neighbor, 2);

    hor_resolution = (hor_fov * M_PI/180.0f)/float(hor_pixel_num);
    ver_resolution = ((ver_max-ver_min) * M_PI/180.0f)/float(ver_pixel_num);

    OnInitialization();

    ros::Subscriber subProjectedPoints = nh.subscribe<sensor_msgs::PointCloud2>("/nv_liom/projected_cloud", 100, OnSubscribeProjectedPointCloud);
    pubNormalPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/nv_liom/normal_vector_cloud", 1000);

    ROS_INFO("\033[1;34mLiDAR Normal Extraction Node Started\033[0m");

    ros::spin();

    return 0;
}