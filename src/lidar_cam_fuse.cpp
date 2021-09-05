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
    double tmp_matrix[4][1] = {point.x, point.y, point.z, 1};
    cv::Mat homo_coordinate(4, 1, CV_64F, tmp_matrix);

    // Calculate the projection of point cloud on image (u, v)
    cv::Mat result = intrinsic_mat * extrinsic_mat * homo_coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);

    uv[0] = int(u / depth);
    uv[1] = int(v / depth);

    // Assign -1 to points behind the camera to ignore these points later
    if (depth < 0)
    {
        uv[0] = -1;
        uv[1] = -1;
    }
}

void get_color(const cv::Mat &img, const int (&uv)[2], int (&rgb)[3])
{
    // Points ouside camera's FOV should be ignored
    if (uv[0] >= 0 && uv[1] >= 0 && uv[0] < img.size().height && uv[1] < img.size().width)
    {
        // Image read by opencv is in BGR format
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
    int uv[2];
    int rgb[3];

    for (pcl::PointXYZRGB &point : cloud)
    {
        // Ignore points at the position of Lidar which are unreasonable
        if (point.x == 0 && point.y == 0 && point.z == 0)
            continue;

        uv[0] = 0;
        uv[1] = 0;
        get_uv(intrinsic_mat, extrinsic_mat, point, uv);

        // Points' default color is grey to be clearly visible
        rgb[0] = 125;
        rgb[1] = 125;
        rgb[2] = 125;
        get_color(img, uv, rgb);

        color_point(point, rgb);
    }
}
