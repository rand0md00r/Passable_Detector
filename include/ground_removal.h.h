#ifndef GROUNDREMOVAL_H_
#define GROUNDREMOVAL_H_

#include <pcl/io/io.h>

const int num_channel = 80;
const int num_bin = 120;
const float r_min = 3.4;
const float r_max = 120;
const float h_min = -2.0;
const float h_max = 1.0;
const float h_diff = 0.4;
const float h_sensor = 2.0;


class Cell
{
    public:
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    public:
    Cell();
    void updateMinZ(float z);
    void updateHeight(float h){height_ = h;}
    void updateSmoothed(float s){smoothed_ = s;}
    void updateHDIFF(float hd){h_diff_ = hd;}
    void updateGround(){is_ground_ = true; h_ground_ = height_;}
    bool isGround(){return is_ground_;}
    float getMinZ(){return min_z_;}
    float getHeight(){return height_;}
    float getHDiff(){return h_diff_;}
    float getSmoothed(){return smoothed_;}
    float getHGround(){return h_ground_;}

    private:
    float smoothed_;
    float height_;
    float h_diff_;
    float h_ground_;
    float min_z_;
    bool is_ground_;

};

class GroundRemoval
{
    public:
    typedef pcl::PointXYZI Point;
    typedef pcl::PointCloud<Point> PointCloud;

    private:
    PointCloud::Ptr raw_cloud_ptr_;
    PointCloud::Ptr elevated_cloud_ptr_;
    PointCloud::Ptr ground_cloud_ptr_;


    
    public:
    GroundRemoval();
    explicit GroundRemoval(std::string pcd_file);
    void readPCDFile(std::string pcd_file);
    void process();
    void filterCloud(PointCloud::Ptr raw_cloud_ptr, PointCloud::Ptr filtered_cloud_ptr);
    void getCellIndex(float x, float y, int& index_channel, int& index_bin);
    void createAndMapPolarGrid(PointCloud::Ptr& cloud_ptr, std::array<std::array<Cell, num_bin>, num_channel>& polar_data);
    void computeHDiffAdjacentCell(std::array<Cell, num_bin>& channel_data);
    void medianFilter(std::array<std::array<Cell, num_bin>, num_channel>& polar_data);
    void outlierFilter(std::array<std::array<Cell, num_bin>, num_channel>& polar_data);
    void visualizationGroundRemoval();
    void saveToPCD(std::string ground_cloud_file, std::string no_ground_cloud_file);
};

#endif