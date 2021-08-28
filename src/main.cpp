#include "ros_bag_load.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;

    std::vector<std::string> topics = {"/camera/tricam/short/image_raw/compressed", "/lidar/points_raw"};
    export_bag("/home/ductm/ros1_bags/sample.bag", topics);
    ros::spin();
}