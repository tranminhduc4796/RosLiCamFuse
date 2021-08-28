#include "ros_bag_load.h"
#include "common.h"

// Callback to process synchornized messages
void process_msgs_cb(const sensor_msgs::CompressedImage::ConstPtr &img_msg, const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    // Convert CompressedImage msg to OpenCV matrix
    // cv::Mat decompressed_img = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_COLOR);2
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
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

    // Export image
    // Export pcl
}

void export_bag(const string &bag_path, const std::vector<std::string> &topics)
{
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    // topics[0]: topic of camera, topics[1]: topic of lidar
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Create fake subscribers to get message from bag to sync
    BagSubscriber<sensor_msgs::CompressedImage> cam_sub;
    BagSubscriber<sensor_msgs::PointCloud2> lidar_sub;

    message_filters::TimeSynchronizer<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> synchronizer(cam_sub, lidar_sub, 30);
    synchronizer.registerCallback(boost::bind(&process_msgs_cb, _1, _2));

    // Loop through each message in the bag
    for (rosbag::MessageInstance m : view)
    {
        // If msg from camera's topic
        if (m.getTopic() == topics[0] || "/" + m.getTopic() == topics[0])
        {
            sensor_msgs::CompressedImage::ConstPtr img_msg = m.instantiate<sensor_msgs::CompressedImage>();

            if (img_msg != NULL)
            {
                // Create fake message to subscriber
                cam_sub.newMessage(img_msg);
            }
        }
        // If msg from lidar's topic
        else if (m.getTopic() == topics[1] || "/" + m.getTopic() == topics[1])
        {
            sensor_msgs::PointCloud2::ConstPtr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (lidar_msg != NULL)
            {
                // Create fake message to subscriber
                lidar_sub.newMessage(lidar_msg);
            }
        }       
    }
    bag.close();
}
