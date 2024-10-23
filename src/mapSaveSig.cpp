#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>

ros::Publisher pubMsg;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "nv_map_save_sig_node");
    ros::NodeHandle nh;
    pubMsg = nh.advertise<std_msgs::Bool>("/nv_liom/end_mapping", 1);

    std::cout<<"Enter 1 to save & exit mapper"<<std::endl;
    int answer;
    std::cout<<"Answer: ";
    std::cin>>answer;

    if(answer != 1) {
        return -1;
    }
    std::cout<<"Exiting"<<std::endl;

    std_msgs::Bool boolMsg;
    boolMsg.data = true;
    pubMsg.publish(boolMsg);

    return 0;
}