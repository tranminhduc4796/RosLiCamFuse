#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

using namespace std;

// Undistort image before projection to get correct color
void undistort_img(cv::Mat &img, const cv::Mat &intrinsic_mat, const cv::Mat &distort_coef);
// Mapping from point (x, y, z) to image pixel (u, v)
void get_uv(const cv::Mat &intrinsic_mat, const cv::Mat &extrinsic_mat, const pcl::PointXYZRGB &point, int (&uv)[2]);
// Get RGB of a pixel (u, v)
void get_color(const cv::Mat &img, const int (&uv)[2], int (&rgb)[3]);
// Color a point of point cloud
void color_point(pcl::PointXYZRGB &point, const int (&rgb)[3]);
// Color the point cloud
void color_cloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat &img, const cv::Mat &intrinsic_mat, const cv::Mat &extrinsic_mat);