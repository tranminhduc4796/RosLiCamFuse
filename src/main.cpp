#include <experimental/filesystem>
#include <pcl/visualization/cloud_viewer.h>

#include "common.h"
#include "ros_bag_load.h"
#include "lidar_cam_fuse.h"

namespace fs = std::experimental::filesystem;

int main(int argc, char **argv)
{
    // Export .jpeg and .pcd from ros bag
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

    int f_count;
    fs::path dir(out_dir_path);

    f_count = count_file(out_dir_path) / 2;

    // Put your data into out_dir if data is not read from ros bag
    if (f_count <= 0)
    {
        std::vector<std::string> topics = {cam_topic, lidar_topic};
        export_bag(bag_path, topics, out_dir_path);

        // Update count after export files to loop through exported files in next process
        f_count = count_file(out_dir_path) / 2;
    }

    // Load calibration params
    double tmp_ex_arr[3][4];
    double tmp_in_arr[3][3];
    double tmp_dist_arr[5][1];

    std::string extrin_path;
    std::string intrin_path;
    std::string distort_path;

    nh.getParam("extrinsic", extrin_path);
    nh.getParam("intrinsic", intrin_path);
    nh.getParam("distortion", distort_path);

    read_extrin(extrin_path, tmp_ex_arr);
    read_intrin(intrin_path, tmp_in_arr);
    read_distort(distort_path, tmp_dist_arr);

    cv::Mat extrin_mat(3, 4, CV_64F, tmp_ex_arr);
    cv::Mat intrin_mat(3, 3, CV_64F, tmp_in_arr);
    cv::Mat dist_coef(5, 1, CV_64F, tmp_dist_arr);

    std::cout << "Extrinsic:\n"
              << cv::format(extrin_mat, cv::Formatter::FMT_NUMPY) 
              << std::endl
              << std::endl;
    std::cout << "Intrinsic:\n"
              << cv::format(intrin_mat, cv::Formatter::FMT_NUMPY) 
              << std::endl
              << std::endl;
    std::cout << "Distortion:\n"
              << cv::format(dist_coef, cv::Formatter::FMT_NUMPY) 
              << std::endl
              << std::endl;

    for (int i = 0; i < f_count; i++)
    {
        // Read .jpg
        fs::path cam_filename("cam_" + std::to_string(i) + ".jpg");
        fs::path cam_filepath = dir / cam_filename;

        cv::Mat img;
        load_img(cam_filepath.string(), img);

        std::cout << "Loaded image with rows x column: "
            << img.rows << " x " << img.cols 
            << std::endl;

        // Read.pcd
        fs::path lidar_filename("lidar_" + std::to_string(i) + ".pcd");
        fs::path lidar_filepath = dir / lidar_filename;

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        load_pcd(lidar_filepath.string(), cloud);

        // Color point cloud
        undistort_img(img, intrin_mat, dist_coef);
        color_cloud(cloud, img, intrin_mat, extrin_mat);

        // Write .pcd
        fs::path out_filename("out_" + std::to_string(i) + ".pcd");
        fs::path out_filepath = dir / out_filename;

        pcl::io::savePCDFileBinaryCompressed(out_filepath.string(), cloud);
    }

    ros::shutdown();

    return 0;
}