#include "lidar_cam_proj.h"
#include "common.h"

// void load_data_from_bag(const string &bag_path, string &lidar_topic, string &camera_topic)
// {
//     ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
//     rosbag::Bag bag;
//     try {
//         bag.open(bag_path, rosbag::bagmode::Read);
//     } catch (rosbag::BagException e) {
//         ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
//         return;
//     }

//     std::vector<string> topics ;
//     topics.push_back(lidar_topic);
//     topics.push_back(camera_topic);

//     rosbag::View view(bag, rosbag::TopicQuery(topics));
// }

void get_uv(const cv::Mat &intrinsic_mat, const cv::Mat &extrinsic_mat, const pcl::PointXYZRGB &point, float (&uv)[2])
{
    // From euclidean to homogeneous coordinate to apply transform
    double matrix3[4][1] = {point.x, point.y, point.z, 1};
    cv::Mat homo_coordinate(4, 1, CV_64F, matrix3);

    // Calculate the projection of point cloud on image (u, v)
    cv::Mat result = intrinsic_mat * extrinsic_mat * homo_coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);

    uv[0] = u / depth;
    uv[1] = v / depth;
}

void get_color(const cv::Mat &img, const float *uv, int (&rgb)[3])
{
    // Note: Read image by opencv is in BGR format
    rgb[0] = img.at<cv::Vec3b>(uv[0], uv[1])[2];
    rgb[1] = img.at<cv::Vec3b>(uv[0], uv[1])[1];
    rgb[2] = img.at<cv::Vec3b>(uv[0], uv[1])[0];
}

void color_point(pcl::PointXYZRGB &point, const int (&rgb)[3])
{
    point.r = rgb[0];
    point.g = rgb[1];
    point.b = rgb[2];
}

void color_cloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat &img, const cv::Mat &intrinsic_mat, const cv::Mat &extrinsic_mat)
{
    // Loop through each point in point cloud
    for (pcl::PointXYZRGB &point : cloud)
    {
        // Ignore invalid point (Just naive implement first)
        if (point.x == 0 && point.y == 0 && point.z == 0)
        {
            continue;
        }

        float uv[2] = {0, 0};
        get_uv(intrinsic_mat, extrinsic_mat, point, uv);

        int rgb[3] = {0, 0, 0};
        get_color(img, uv, rgb);
        // Ignore points without color
        if (rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 0)
        {
            continue;
        }

        color_point(point, rgb);
    }
}

int main()
{
    // Extrinsic matrix (3, 4)
    double tmp_ex_arr[3][4] =
        {{7.533745000000e-03, -9.999714000000e-01, -6.166020000000e-04, -4.069766000000e-03},
         {1.480249000000e-02, 7.280733000000e-04, -9.998902000000e-01, -7.631618000000e-02},
         {9.998621000000e-01, 7.523790000000e-03, 1.480755000000e-02, -2.717806000000e-01}};
    cv::Mat extrin_mat(3, 4, CV_64F, tmp_ex_arr);

    // Intrinsic matrix (3, 3)
    double tmp_in_arr[3][3] =
        {{1, 0, 0},
         {0, 1, 0},
         {0, 0, 1}};
    cv::Mat intrin_mat(3, 3, CV_64F, tmp_in_arr);

    // Read .pcd file
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    load_pcd("sample.pcd", cloud);

    // Read image file
    cv::Mat img;
    load_img("sample.jpg", img);

    color_cloud(cloud, img, intrin_mat, extrin_mat);
};