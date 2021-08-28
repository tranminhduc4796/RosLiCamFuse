#include "ros_bag_load.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fuse");
    ros::NodeHandle nh("~");

    std::string out_dir_path;
    std::string bag_path;
    std::string lidar_topic;
    std::string cam_topic;

    nh.getParam("bag", bag_path);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("cam_topic", cam_topic);
    nh.getParam("out_dir", out_dir_path);
    
    std::vector<std::string> topics = {cam_topic, lidar_topic};
    export_bag(bag_path, topics, out_dir_path);
    ros::spin();
}