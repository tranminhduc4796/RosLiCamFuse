#ifndef COMMON_H
#define COMMON_H

#include <fstream>
#include <experimental/filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

void load_pcd(const std::string &filepath, pcl::PointCloud<pcl::PointXYZRGB> &cloud);

void load_img(const std::string &filepath, cv::Mat &img);

double str2double(std::string &str);

void read_intrin(const std::string &filepath, double (&intrin_mat)[3][3]);

void read_extrin(const std::string &filepath, double (&extrin_mat)[3][4]);

void read_distort(const std::string &filepath, double (&distort_vec)[5][1]);

int count_file(const std::string &dirpath);

#endif // COMMON_h
