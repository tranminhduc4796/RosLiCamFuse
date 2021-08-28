#ifndef COMMON_H
#define COMMON_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

void load_pcd(const std::string &filepath, pcl::PointCloud<pcl::PointXYZRGB> &cloud);

void load_img(const std::string &filepath, cv::Mat &img);

#endif // COMMON_h
