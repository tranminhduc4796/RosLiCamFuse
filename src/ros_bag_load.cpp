#include <experimental/filesystem>

#include "ros_bag_load.h"
#include "common.h"

using namespace sensor_msgs;
namespace fs =  std::experimental::filesystem;

int PAIR_COUNT = 0;

// Callback to process synchornized messages
void process_msgs_cb(const CompressedImage::ConstPtr &img_msg, const PointCloud2::ConstPtr &cloud_msg, const std::string &out_dir_path)
{
    // Convert CompressedImage msg to OpenCV matrix
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, image_encodings::BGR8);
    }
    catch(const cv_bridge::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return;
    }

    // Convert PointCloud2 msg to PCL Point Cloud
    pcl::PCLPointCloud2 pc2_cloud;
    pcl_conversions::toPCL(*cloud_msg, pc2_cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pc2_cloud, cloud);

    fs::path dir(out_dir_path);
    // Export .jpg
    fs::path cam_filename("cam_" + std::to_string(PAIR_COUNT) + ".jpg");
    fs::path cam_filepath = dir / cam_filename;
    cv::imwrite(cam_filepath.string(), cv_ptr->image);

    // Export .pcl
    fs::path lidar_filename("lidar_" + std::to_string(PAIR_COUNT) + ".pcd");
    fs::path lidar_filepath = dir / lidar_filename;
    pcl::io::savePCDFileBinaryCompressed(lidar_filepath.string(), cloud);
    PAIR_COUNT++;
}

void export_bag(const string &bag_path, const std::vector<std::string> &topics, const std::string &out_dir_path)
{
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    // topics[0]: topic of camera, topics[1]: topic of lidar
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Create fake subscribers to get message from bag to sync
    BagSubscriber<CompressedImage> cam_sub;
    BagSubscriber<PointCloud2> lidar_sub;

    // Use time approximate synchronizer
    typedef message_filters::sync_policies::ApproximateTime<CompressedImage, PointCloud2> sync_policy;
    message_filters::Synchronizer<sync_policy> synchronizer(sync_policy(10), cam_sub, lidar_sub);
    synchronizer.registerCallback(boost::bind(&process_msgs_cb, _1, _2, out_dir_path));

    // Loop through each message in the bag
    for (rosbag::MessageInstance m : view)
    {
        // If msg from camera's topic
        if (m.getTopic() == topics[0] || "/" + m.getTopic() == topics[0])
        {
            CompressedImage::ConstPtr img_msg = m.instantiate<CompressedImage>();

            if (img_msg != NULL)
            {
                // Create fake message to subscriber
                cam_sub.newMessage(img_msg);
            }
        }
        // If msg from lidar's topic
        else if (m.getTopic() == topics[1] || "/" + m.getTopic() == topics[1])
        {
            PointCloud2::ConstPtr lidar_msg = m.instantiate<PointCloud2>();
            if (lidar_msg != NULL)
            {
                // Create fake message to subscriber
                lidar_sub.newMessage(lidar_msg);
            }
        }       
    }
    bag.close();
}
