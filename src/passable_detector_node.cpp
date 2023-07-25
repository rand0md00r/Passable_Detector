#include "passable_detector_node.h"


PassableDetector::PassableDetector(ros::NodeHandle& nh)
    : pnh_(), 
    raw_cloud_ptr_(new PointCloud), 
    elevated_cloud_ptr_(new PointCloud), 
    ground_cloud_ptr_(new PointCloud)
{
    // Params
    pnh_.param("thread_nums", theread_num_, 2);// number of threads for this ROS node
    pnh_.param("frequency",   frequency_,  10);
    pnh_.param("num_channel", num_channel_, 80);
    pnh_.param("num_bin",     num_bin_,     120);
    pnh_.param("r_min",       r_min,       0.3);
    pnh_.param("r_max",       r_max,       10.0);
    pnh_.param("h_min",       h_min,       -2.0);
    pnh_.param("h_max",       h_max,       1.0);
    pnh_.param("h_diff",      h_diff,      0.01);
    pnh_.param("h_sensor",    h_sensor,    0.5);

    // Sub && Pub
    cloud_sub_  = pnh_.subscribe("/velodyne_points2", 1, &PassableDetector::pointCloudCallback, this);
    map_sub_    = pnh_.subscribe("/map", 1, &PassableDetector::mapCallback, this);

    map_pub_    = pnh_.advertise<nav_msgs::OccupancyGrid>("/passable_grid", 1);
    ground_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
    elevat_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("/elevated_cloud", 1);
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("/obstacle", 1);


    // Timer for process
    processer_  = pnh_.createTimer(ros::Duration((1.0) / frequency_), &PassableDetector::process, this);

    // Params init
    received_cloud_ = false;
    received_map_   = false;
}

std::vector<double> gaussKernel(int samples, double sigma)
{
    std::vector<double> kernel(samples);
    double mean = samples/2;
    double sum = 0.0;
    for(int x=0; x < samples; ++x)
    {
        kernel[x] = exp( -0.5* (pow((x-mean)/sigma, 2.0)) /  (2*M_PI*sigma*sigma));
        sum += kernel[x];
    }

    for(auto& kernel_value : kernel)
    {
        kernel_value /= sum;
    }

    assert(kernel.size() == samples);

    return kernel;
}

void gaussSmoothen(std::array<Cell,num_bin>& values, double sigma, int samples)
{
    auto kernel = gaussKernel(samples, sigma);
    int sample_side = samples / 2;
    size_t ubound = values.size();
    for(size_t i = 0; i< ubound; ++i)
    {
        double smoothed = 0;
        for(size_t j = i-sample_side; j<= i+sample_side; ++j)
        {
            if(j >= 0 && i< ubound)
            {
                // 近的权重大
                int weight_index = sample_side + (j-i);
                smoothed += kernel[weight_index] * values[j].getHeight();
            }
        }
        values[i].updateSmoothed(smoothed);
    }
}


void PassableDetector::process(const ros::TimerEvent&)
{
    // Filter
    PointCloud::Ptr filtered_cloud_ptr(new PointCloud);
    filterCloud(raw_cloud_ptr_, filtered_cloud_ptr);

    std::array<std::array<Cell, num_bin>, num_channel> polar_data;
    createAndMapPolarGrid(filtered_cloud_ptr, polar_data);

    // 阈值筛选
    for(int index_channel = 0; index_channel < num_channel; ++index_channel)
    {
        for(int index_bin = 0; index_bin < num_bin; ++index_bin)
        {
            float z_min = polar_data[index_channel][index_bin].getMinZ();
            if(z_min > h_min && z_min < h_max)
            {
                polar_data[index_channel][index_bin].updateHeight(z_min);
            }

            else if(z_min > h_max)
            {
                polar_data[index_channel][index_bin].updateHeight(h_sensor);
            }

            else
            {
                polar_data[index_channel][index_bin].updateHeight(h_min);
            }
        }

        // 高斯平滑
        gaussSmoothen(polar_data[index_channel], 1, 3);
        // 算同一channel邻接bin的height差
        computeHDiffAdjacentCell(polar_data[index_channel]);

        // 判断是否ground  高度正常、高度突变正常
        for(int index_bin = 0; index_bin < polar_data[0].size(); ++index_bin)
        {
            if(polar_data[index_channel][index_bin].getHeight() < h_max&& polar_data[index_channel][index_bin].getHDiff() < h_diff)
            {
                polar_data[index_channel][index_bin].updateGround();
            }

            else if(polar_data[index_channel][index_bin].getSmoothed() < h_max&& polar_data[index_channel][index_bin].getHDiff() < h_diff)
            {
                polar_data[index_channel][index_bin].updateGround();
            }
        }
    }

    medianFilter(polar_data);
    outlierFilter(polar_data);

    for(auto& point : filtered_cloud_ptr->points)
    {
        int index_channel, index_bin;
        getCellIndex(point.x, point.y, index_channel, index_bin);

        if(index_channel < 0 || index_channel > num_channel || index_bin < 0 || index_bin > num_bin){continue;}

        if(polar_data[index_channel][index_bin].isGround())
        {
            float h_ground = polar_data[index_channel][index_bin].getHGround();
            if(point.z < (h_ground+0.25))
            {
                ground_cloud_ptr_->points.push_back(point);
            }
            else
            {
                elevated_cloud_ptr_->points.push_back(point);
            }
        }
        else{
            elevated_cloud_ptr_->points.push_back(point);
        }
    }

    // Publish
    projectPointCloudToMap();
    publishGroundCloud();
    publishElevatCloud();
    publishFusedMap();
    visualizeCubes();
}

int PassableDetector::getThreadNumbers()
{
    return theread_num_;
}

void PassableDetector::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::fromROSMsg(*cloud_msg, *raw_cloud_ptr_);
    frame_id_ = cloud_msg->header.frame_id;
    received_cloud_ = true;
}

void PassableDetector::mapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
{
    raw_map_ = *map_msg;
    received_map_ = true;

    // Set fused map
    fused_map_.header.frame_id = "base_link";
    fused_map_.info.height = map_msg->info.height;
    fused_map_.info.width = map_msg->info.width;
    fused_map_.info.resolution = map_msg->info.resolution;
    fused_map_.info.origin = map_msg->info.origin;
}

void PassableDetector::publishGroundCloud()
{
    if (ground_cloud_ptr_->empty() && received_cloud_) {
        ROS_WARN("The ground cloud is empty, not publishing");
        return;
    }
    
    sensor_msgs::PointCloud2 ground_cloud_msg;
    pcl::toROSMsg(*ground_cloud_ptr_, ground_cloud_msg);

    ground_cloud_msg.header.stamp = ros::Time::now();
    ground_cloud_msg.header.frame_id = frame_id_;  // replace this with the correct frame
    ground_pub_.publish(ground_cloud_msg);
    ground_cloud_ptr_->clear();
}

void PassableDetector::publishElevatCloud()
{
    if (elevated_cloud_ptr_->empty() && received_cloud_) {
        ROS_WARN("The elevated cloud is empty, not publishing");
        return;
    }

    sensor_msgs::PointCloud2 elevated_cloud_msg;
    pcl::toROSMsg(*elevated_cloud_ptr_, elevated_cloud_msg);

    elevated_cloud_msg.header.stamp = ros::Time::now();
    elevated_cloud_msg.header.frame_id = frame_id_;  // replace this with the correct frame
    elevat_pub_.publish(elevated_cloud_msg);
    elevated_cloud_ptr_->clear();
}

void PassableDetector::publishFusedMap()
{
    if (fused_map_.data.empty() && received_map_)
    {
        ROS_WARN("Attempted to publish empty fused map.");
        return;
    }

    if (fused_map_.info.width * fused_map_.info.height != fused_map_.data.size()) 
    {
        ROS_ERROR("Map size does not match width and height.");
        return;
    }

    map_pub_.publish(fused_map_);
}


void PassableDetector::projectPointCloudToMap() {
    // 获取地图的分辨率
    double map_resolution = fused_map_.info.resolution;

    // 获取地图的原点
    double origin_x = fused_map_.info.origin.position.x;
    double origin_y = fused_map_.info.origin.position.y;

    // 清空地图数据
    fused_map_.data.clear();

    // 地图的尺寸
    int map_width = fused_map_.info.width;
    int map_height = fused_map_.info.height;

    // 初始化地图数据
    fused_map_.data.resize(map_width * map_height, -1);  // -1 表示未知

    // 遍历点云数据
    for (const auto& point : *ground_cloud_ptr_) {
        // 计算点云数据在地图上的位置
        int x = round((point.x - origin_x) / map_resolution);
        int y = round((point.y - origin_y) / map_resolution);

        // 检查点是否在地图范围内
        if (x >= 0 && x < map_width && y >= 0 && y < map_height) {
            // 将该位置标记为已占用
            fused_map_.data[y * map_width + x] = 0;  // 100 表示已占用
        }
    }
}


/**
 * \brief 滤除范围外的点
 */ 
void PassableDetector::filterCloud(PointCloud::Ptr raw_cloud_ptr, PointCloud::Ptr filtered_cloud_ptr)
{
    for(auto& point:raw_cloud_ptr->points)
    {
        float x = point.x;
        float y = point.y;
        float z = point.z;
        float dis = sqrt(x*x + y*y);
        if(dis <= r_min)
        {
            continue;
        }
        else
        {
            filtered_cloud_ptr->points.push_back(point);
        }
    }
}


/**
 * \brief 映射到网格极坐标上
 */ 
void PassableDetector::createAndMapPolarGrid(PointCloud::Ptr& cloud_ptr, std::array<std::array<Cell, num_bin>, num_channel>& polar_data)
{
    for(auto& point:cloud_ptr->points)
    {
        float x = point.x;
        float y = point.y;
        float z = point.z;

        int index_channel, index_bin;
        getCellIndex(x, y, index_channel, index_bin);
        if(index_channel <0 || index_channel > num_channel || index_bin <0 || index_bin > num_bin)
        {
            continue;
        }
        polar_data[index_channel][index_bin].updateMinZ(z);
    }
}

/**
 * \brief x,y -> index_channel, index_bin
 */ 
void PassableDetector::getCellIndex(float x, float y, int& index_channel, int& index_bin)
{
    float dis = sqrt(x*x + y*y);
    float polar_channel = (atan2(y,x)+M_PI) / (2*M_PI);
    float polar_bin = (dis-r_min) / (r_max-r_min);
    index_channel = floor(polar_channel*num_channel);
    index_bin = floor(polar_bin*num_bin);
}

/**
 * \brief 算高度差
 */ 
void PassableDetector::computeHDiffAdjacentCell(std::array<Cell, num_bin>& channel_data)
{
        for(int i=0; i< channel_data.size(); ++i)
    {
        if(i==0)
        {
            float h_diff_temp = channel_data[i].getHeight() - channel_data[i+1].getHeight();
            channel_data[i].updateHDIFF(h_diff_temp);
        }
        else if(i==channel_data.size()-1)
        {
            float h_diff_temp = channel_data[i].getHeight() - channel_data[i-1].getHeight();
            channel_data[i].updateHDIFF(h_diff_temp);
        }
        else
        {
            float pre_diff_temp = channel_data[i].getHeight() - channel_data[i-1].getHeight();
            float post_diff_temp = channel_data[i].getHeight() - channel_data[i+1].getHeight();

            if(pre_diff_temp > post_diff_temp){channel_data[i].updateHDIFF(pre_diff_temp);}
            else{channel_data[i].updateHDIFF(post_diff_temp);}
        }
    }
}

/**
 * \brief 中值滤波处理缺失的地面信息
 */ 
void PassableDetector::medianFilter(std::array<std::array<Cell, num_bin>, num_channel>& polar_data)
{
    for(int index_channel = 1; index_channel < polar_data.size()-1; ++index_channel)
    {
        for(int index_bin = 1; index_bin < polar_data[0].size()-1; ++index_bin)
        {
            if(!polar_data[index_channel][index_bin].isGround())
            {
                if(polar_data[index_channel][index_bin+1].isGround()&&
                polar_data[index_channel][index_bin-1].isGround()&&
                polar_data[index_channel+1][index_bin].isGround()&&
                polar_data[index_channel-1][index_bin].isGround())
                {
                    std::vector<float> adj_cell_vec{polar_data[index_channel][index_bin+1].getHeight(),
                    polar_data[index_channel][index_bin-1].getHeight(),
                    polar_data[index_channel+1][index_bin].getHeight(),
                    polar_data[index_channel-1][index_bin].getHeight()};
                    std::sort(adj_cell_vec.begin(), adj_cell_vec.end());
                    float median = (adj_cell_vec[1] + adj_cell_vec[2]) * 0.5;
                    polar_data[index_channel][index_bin].updateHeight(median);
                    polar_data[index_channel][index_bin].updateGround();
                }
            }
        }
    }
}

/**
 * \brief 对于高度极低的cell的处理
 */ 
void PassableDetector::outlierFilter(std::array<std::array<Cell, num_bin>, num_channel>& polar_data)
{
    for(int index_channel = 1; index_channel < polar_data.size()-1; ++index_channel)
    {
        for(int index_bin = 1; index_bin < polar_data[0].size()-2; ++index_bin)
        {
            if(polar_data[index_channel][index_bin].isGround()&&
            polar_data[index_channel][index_bin-1].isGround()&&
            polar_data[index_channel][index_bin+1].isGround()&&
            polar_data[index_channel][index_bin+2].isGround())
            {
                float height1 = polar_data[index_channel][index_bin-1].getHeight();
                float height2 = polar_data[index_channel][index_bin].getHeight();
                float height3 = polar_data[index_channel][index_bin+1].getHeight();
                float height4 = polar_data[index_channel][index_bin+2].getHeight();

                if(height1!= h_min && height2== h_min && height3!=h_min && height4!=h_min)
                {
                    polar_data[index_channel][index_bin].updateHeight(0.5*(height1+height4));
                    polar_data[index_channel][index_bin].updateGround();
                }
                else if(height1!= h_min && height2== h_min && height3!=h_min)
                {
                    polar_data[index_channel][index_bin].updateHeight(0.5*(height1+height3));
                    polar_data[index_channel][index_bin].updateGround();
                }
            }
        }
    }

}


/**
 * \brief Cell 类实现
 */ 
Cell::Cell():min_z_(1000),is_ground_(false)
{

}

void Cell::updateMinZ(float z)
{
    if(z < min_z_)
    {
        min_z_ = z;
    }
}


/**
 * \brief 聚类点云并返回立方体信息
 */ 
void PassableDetector::clusterPointCloud() {
    // std::vector<CubeInfo> cube_infos;

    // 降采样点云，以提高聚类效率
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setInputCloud(elevated_cloud_ptr_);
    voxel_grid.setLeafSize(0.1, 0.1, 0.1); // 根据需要调整leaf size
    pcl::PointCloud<Point>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<Point>());
    voxel_grid.filter(*downsampled_cloud_ptr);

    // 创建一个KD树对象，用于搜索聚类时的近邻点
    pcl::search::KdTree<Point>::Ptr kd_tree(new pcl::search::KdTree<Point>());
    kd_tree->setInputCloud(downsampled_cloud_ptr);

    // 使用欧几里德聚类算法
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> euclidean_cluster;
    euclidean_cluster.setClusterTolerance(0.5); // 根据需要调整聚类的距离阈值
    euclidean_cluster.setMinClusterSize(10);    // 根据需要调整聚类的最小点数
    euclidean_cluster.setInputCloud(downsampled_cloud_ptr);
    euclidean_cluster.setSearchMethod(kd_tree);
    euclidean_cluster.extract(cluster_indices);
    

    // 遍历聚类结果，获取每个聚类的立方体信息
    for (const auto& cluster_index : cluster_indices) {
        CubeInfo cube_info;

        // 获取聚类的点云子集
        PointCloud::Ptr cluster_cloud(new PointCloud());
        pcl::copyPointCloud(*elevated_cloud_ptr_, cluster_index.indices, *cluster_cloud);

        // 创建MomentOfInertiaEstimation对象
        pcl::MomentOfInertiaEstimation<Point> feature_extractor;
        feature_extractor.setInputCloud(cluster_cloud);
        feature_extractor.compute();

        // 计算聚类的AABB
        Point min_pt, max_pt;
        feature_extractor.getAABB(min_pt, max_pt);

        cube_info.min_point.x = min_pt.x;
        cube_info.min_point.y = min_pt.y;
        cube_info.min_point.z = min_pt.z;
        cube_info.max_point.x = max_pt.x;
        cube_info.max_point.y = max_pt.y;
        cube_info.max_point.z = max_pt.z;

        cube_info.center.x = (cube_info.min_point.x + cube_info.max_point.x) / 2;
        cube_info.center.y = (cube_info.min_point.y + cube_info.max_point.y) / 2;
        cube_info.center.z = (cube_info.min_point.z + cube_info.max_point.z) / 2;
        cube_info.length = cube_info.max_point.x - cube_info.min_point.x;
        cube_info.width = cube_info.max_point.y - cube_info.min_point.y;
        cube_info.height = cube_info.max_point.z - cube_info.min_point.z;
        // 将立方体信息添加到向量中
        cube_infos_.push_back(cube_info);
    }
}

/**
 * \brief 可视化函数，发布立方体信息为MarkerArray消息
 */ 
void PassableDetector::visualizeCubes() {
    visualization_msgs::MarkerArray marker_array;

    int marker_id = 0;
    if(cube_infos_.empty()) return;
    for (const auto& cube_info : cube_infos_) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "cubes";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = cube_info.center.x;
        marker.pose.position.y = cube_info.center.y;
        marker.pose.position.z = cube_info.center.z;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = cube_info.length;
        marker.scale.y = cube_info.width;
        marker.scale.z = cube_info.height;

        marker.color.r = 1.0; // 立方体颜色为红色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5; // 透明度为0.5

        marker.lifetime = ros::Duration(); // 持续时间为永久

        marker_array.markers.push_back(marker);

        ++marker_id;
    }

    marker_pub_.publish(marker_array);
    cube_infos_.clear();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "passable_detector");
    ros::NodeHandle pnh_("~");

    PassableDetector pd(pnh_);

    ros::AsyncSpinner spinner(pd.getThreadNumbers()); // Use multi threads
    spinner.start();

    ros::waitForShutdown();
    return 0;
}
