#ifndef PASSABLE_DETECTOR_NODE_H
#define PASSABLE_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
// #include "gauss.h"

using namespace std;

const int num_channel = 80;
const int num_bin = 120;
const float r_min = 0.3;
const float r_max = 30;
const float h_min = -0.95;
const float h_max = 1.0;
const float h_diff = 0.4;
const float h_sensor = 0.95;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

// 定义一个结构体用于存储立方体信息
struct CubeInfo {
    Point min_point;  // 立方体的最小点
    Point max_point;  // 立方体的最大点
    Point center;     // 立方体的中心点
    float length;            // 立方体的长度
    float width;             // 立方体的宽度
    float height;            // 立方体的高度
};

class Cell
{
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
    int num_channel, num_bin;
};

class PassableDetector
{
private:
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_, map_sub_;
    ros::Publisher map_pub_, ground_pub_, elevat_pub_, marker_pub_;
    ros::Timer processer_;

    PointCloud::Ptr raw_cloud_ptr_;
    PointCloud::Ptr elevated_cloud_ptr_;
    PointCloud::Ptr ground_cloud_ptr_;

    nav_msgs::OccupancyGrid raw_map_, fused_map_;

    bool received_cloud_, received_map_;
    int theread_num_, frequency_, num_channel_, num_bin_;
    double r_min, r_max, h_min, h_max, h_diff, h_sensor;            // for ground remove
    string frame_id_;

    std::vector<CubeInfo> cube_infos_;

public:
    PassableDetector(ros::NodeHandle& nh);

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg);
    void process(const ros::TimerEvent&);
    void publishGroundCloud();
    void publishFusedMap();
    void publishElevatCloud();
    int getThreadNumbers();
    void projectPointCloudToMap();

    void filterCloud(PointCloud::Ptr raw_cloud_ptr, PointCloud::Ptr filtered_cloud_ptr);
    void createAndMapPolarGrid(PointCloud::Ptr& cloud_ptr, std::array<std::array<Cell, num_bin>, num_channel>& polar_data);
    void getCellIndex(float x, float y, int& index_channel, int& index_bin);
    void computeHDiffAdjacentCell(std::array<Cell, num_bin>& channel_data);
    void medianFilter(std::array<std::array<Cell, num_bin>, num_channel>& polar_data);
    void outlierFilter(std::array<std::array<Cell, num_bin>, num_channel>& polar_data);
    
    void clusterPointCloud();
    void visualizeCubes();
};

#endif