// #include <ros/ros.h>
// #include <std_msgs/Header.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

using namespace std;

// Mapping from point (x, y, z) to image pixel (u, v)
void get_uv(const cv::Mat &intrinsic_mat, const cv::Mat &extrinsic_mat, const pcl::PointXYZRGB &point, float (&uv)[2]);
// Get RGB of a pixel (u, v)
void get_color(const cv::Mat &img, const float *uv, int (&rgb)[3]);
// Color a point of point cloud
void color_point(pcl::PointXYZRGB &point, const int (&rgb)[3]);
// Color the point cloud
void color_cloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat &img, const cv::Mat &intrinsic_mat, const cv::Mat &extrinsic_mat);

// Load and synchronize image and pointcloud.
// void load_data_from_bag(const string &bag_path);
// void load_img_from_bag(const rosbag::Bag &bag);
// void load_pc_from_bag(const rosbag::Bag &bag);

// Struct to hold synchronized data
struct CamLidar
{
    cv::Mat img;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
};
