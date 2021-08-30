#include "lidar_cam_fuse.h"
#include "common.h"
#include "pcl/visualization/cloud_viewer.h"


void undistort_img(cv::Mat &img, const cv::Mat &intrinsic_mat, const cv::Mat &distort_coef)
{
    cv::Mat view, rview, map1, map2;
    cv::Size img_size = img.size();

    // Refine the camera matrix to keep all the pixels when undistort
    cv::Mat refined_cam_mat = cv::getOptimalNewCameraMatrix(intrinsic_mat, distort_coef, img_size, 1, img.size(), 0);
    cv::initUndistortRectifyMap(intrinsic_mat, distort_coef, cv::Mat(), refined_cam_mat, img_size, CV_16SC2, map1, map2);

    // Undistort image
    cv::remap(img, img, map1, map2, cv::INTER_LINEAR);
}

void get_uv(const cv::Mat &intrinsic_mat, const cv::Mat &extrinsic_mat, const pcl::PointXYZRGB &point, int (&uv)[2])
{
    // From euclidean to homogeneous coordinate to apply transform
    double matrix3[4][1] = {point.x, point.y, point.z, 1};
    cv::Mat homo_coordinate(4, 1, CV_64F, matrix3);

    // Calculate the projection of point cloud on image (u, v)
    cv::Mat result = intrinsic_mat * extrinsic_mat * homo_coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);

    uv[0] = int(u / depth);
    uv[1] = int(v / depth);

    // Assign -1 to points behind the camera
    if (depth < 0)
    {
        uv[0] = -1;
        uv[1] = -1;
    }
}

void get_color(const cv::Mat &img, const int (&uv)[2], int (&rgb)[3])
{
    // Only care valid points
    if (uv[0] >= 0 && uv[1] >= 0 && uv[0] <= img.size().height && uv[1] <= img.size().width)
    {
        // Note: Read image by opencv is in BGR format
        rgb[0] = img.at<cv::Vec3b>(uv[0], uv[1])[2];
        rgb[1] = img.at<cv::Vec3b>(uv[0], uv[1])[1];
        rgb[2] = img.at<cv::Vec3b>(uv[0], uv[1])[0];
    }
}

void color_point(pcl::PointXYZRGB &point, const int (&rgb)[3])
{
    point.r = rgb[0];
    point.g = rgb[1];
    point.b = rgb[2];
}

void color_cloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat &img, const cv::Mat &intrinsic_mat, const cv::Mat &extrinsic_mat)
{
    int uv[2] = {0, 0};
    int rgb[3] = {0, 0, 0};

    // Loop through each point in point cloud
    for (pcl::PointXYZRGB &point : cloud)
    {
        // Ignore invalid point (Just naive implement first)
        if (point.x == 0 && point.y == 0 && point.z == 0)
            continue;

        uv[0] = 0;
        uv[1] = 0;
        get_uv(intrinsic_mat, extrinsic_mat, point, uv);

        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = 0;
        get_color(img, uv, rgb);

        // Ignore points without color
        if (rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 0)
            continue;

        color_point(point, rgb);
    }
}

// int main()
// {
//     // Extrinsic matrix (3, 4)
//     double tmp_ex_arr[3][4] =
//         {{-0.0090788, 0.0851177, 0.9963295, 0.85383986},
//          {-0.9999462, -0.0057710, -0.0086187, 0.154835},
//          {0.0050163, -0.9963542, 0.0851655, 1.03181427}};
//     cv::Mat extrin_mat(3, 4, CV_64F, tmp_ex_arr);

//     // Intrinsic matrix (3, 3)
//     double tmp_in_arr[3][3] =
//         {{3974.66, 0, 934.31},
//          {0, 3970.66, 575.35},
//          {0, 0, 1.0}};
//     cv::Mat intrin_mat(3, 3, CV_64F, tmp_in_arr);

//     // Distortion coef(5, 1)
//     double tmp_dist_arr[5][1] =
//         {{-0.328462},
//          {0.078086},
//          {-0.004439},
//          {-0.000034},
//          {0.000000}};
//     cv::Mat dist_coef(5, 1, CV_64F, tmp_dist_arr);

//     // Read image file
//     cv::Mat img;
//     load_img("/home/ductm/catkin_ws/src/lidar_cam_proj/img/sample.jpg", img);
//     std::cout << "Image size: ";
//     std::cout << img.size().width << ", ";
//     std::cout << img.size().height << std::endl;

//     // Read .pcd file
//     pcl::PointCloud<pcl::PointXYZRGB> cloud;
//     load_pcd("/home/ductm/catkin_ws/src/lidar_cam_proj/pcd/sample.pcd", cloud);

//     undistort_img(img, intrin_mat, dist_coef);
//     color_cloud(cloud, img, intrin_mat, extrin_mat);

//     // Visualize point cloud
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(&cloud);
//     pcl::visualization::CloudViewer viewer("RGB Point Cloud Viewer");
//     viewer.showCloud(cloud_ptr);
//     while (!viewer.wasStopped())
//     {
//     }
// }