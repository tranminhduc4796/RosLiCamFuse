#include "common.h"

void load_pcd(const std::string &filepath, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filepath, cloud) == -1) //* load the file
    {
        std::cerr << "Couldn't read .pcd file \n";
        std::exit(1);
    }
}

void load_img(const std::string &filepath, cv::Mat &img)
{
    img = cv::imread(filepath, cv::IMREAD_COLOR);
    if (img.empty())
    {
        std::cerr << "Coulnd't read image file \n";
        std::exit(1);
    }
}