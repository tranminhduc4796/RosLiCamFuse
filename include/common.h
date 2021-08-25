#ifndef COMMON_H
#define COMMON_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

//  Load .pcd file
void load_pcd(const std::string &filepath, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filepath, cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read .pcd file \n");
    }
}

void load_img(const std::string &filepath, cv::Mat &img)
{
    img = cv::imread(filepath, cv::IMREAD_COLOR);
    if (img.empty())
    {
        std::cerr << "Coulnd't read image file \n";
        throw "Coulnd't read image file \n";
    }
}
#endif // COMMON_h