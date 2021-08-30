#include "common.h"

namespace fs = std::experimental::filesystem;

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

double str2double(std::string &str)
{
    double d;
    std::stringstream sin(str);
    if (sin >> d)
    {
        return d;
    }
    std::cerr << str << std::endl;
    std::cout << "Can not convert a string to double" << std::endl;
    exit(0);
}

void read_intrin(const std::string &filepath, double (&intrin_mat)[3][3])
{
    std::string line;
    std::ifstream f(filepath);

    if (f.is_open())
    {
        for (int i = 0; i < 3; i++)
        {
            // Read a line from the file
            std::getline(f, line);
            // Split line into words
            std::stringstream words(line);
            std::string word;
            
            for (int j = 0; j < 3; j++)
            {
                // Pass each word to variable word
                words >> word;
                intrin_mat[i][j] = str2double(word);
            }
        }
    }
}

void read_extrin(const std::string &filepath, double (&extrin_mat)[3][4])
{
    std::string line;
    std::ifstream f(filepath);

    if (f.is_open())
    {
        for (int i = 0; i < 3; i++)
        {
            // Read a line from the file
            std::getline(f, line);
            // Split line into words
            std::stringstream words(line);
            std::string word;
            
            for (int j = 0; j < 4; j++)
            {
                // Pass each word to variable word
                words >> word;
                extrin_mat[i][j] = str2double(word);
            }
        }
    }
}

void read_distort(const std::string &filepath, double (&distort_vec)[5][1])
{
    std::string line;
    std::ifstream f(filepath);

    if (f.is_open())
    {
        std::getline(f, line);
        std::stringstream words(line);
        std::string word;

        for (int i = 0; i < 5; i++)
        {
            words >> word;
            distort_vec[i][0] = str2double(word);
        }
    }
}

int count_file(const std::string &dir_path)
{
    fs::path dir(dir_path);
    int count = 0;

    for (auto f : fs::directory_iterator(dir))
    {
        count++;
    }
    return count;
}