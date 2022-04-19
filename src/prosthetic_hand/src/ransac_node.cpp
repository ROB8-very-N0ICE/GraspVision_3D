#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ransac_node");
    ros::Rate r(5);
    ros::NodeHandle nh;
    ros::spin();
}
