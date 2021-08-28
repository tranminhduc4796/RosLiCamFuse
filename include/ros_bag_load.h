#ifndef ROS_BAG_LOAD_H
#define ROS_BAG_LOAD_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>


using namespace std;

// Load, synchronize image and pointcloud, and export data.
void export_bag(const string &bag_path, const std::vector<std::string> &topics);

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    this->signalMessage(msg);
  }
};

#endif  // ROS_BAG_LOAD_H
