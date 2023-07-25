#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "GroundRemoval.h"
#include "Gauss.h"

Cell::Cell():min_z_(1000),is_ground_(false)
{}

void Cell::updateMinZ(float z)
{
    if(z < min_z_)
    {
        min_z_ = z;
    }
}

GroundRemoval::GroundRemoval():raw_cloud_ptr_(new PointCloud), elevated_cloud_ptr_(new PointCloud), ground_cloud_ptr_(new PointCloud)
{

}

/**
 * \brief 滤除范围外的点
 */ 
void GroundRemoval::filterCloud(PointCloud::Ptr raw_cloud_ptr, PointCloud::Ptr filtered_cloud_ptr)
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
 * \brief x,y -> index_channel, index_bin
 */ 
void GroundRemoval::getCellIndex(float x, float y, int& index_channel, int& index_bin)
{
    float dis = sqrt(x*x + y*y);
    float polar_channel = (atan2(y,x)+M_PI) / (2*M_PI);
    float polar_bin = (dis-r_min) / (r_max-r_min);
    index_channel = floor(polar_channel*num_channel);
    index_bin = floor(polar_bin*num_bin);
}


/**
 * \brief 映射到网格极坐标上
 */ 
void GroundRemoval::createAndMapPolarGrid(PointCloud::Ptr& cloud_ptr, std::array<std::array<Cell, num_bin>, num_channel>& polar_data)
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
 * \brief 算高度差
 */ 
void GroundRemoval::computeHDiffAdjacentCell(std::array<Cell, num_bin>& channel_data)
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
void GroundRemoval::medianFilter(std::array<std::array<Cell, num_bin>, num_channel>& polar_data)
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
void GroundRemoval::outlierFilter(std::array<std::array<Cell, num_bin>, num_channel>& polar_data)
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



void GroundRemoval::process()
{
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
}


void GroundRemoval::visualizationGroundRemoval()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<Point>(ground_cloud_ptr_,"ground_cloud");
    viewer->addPointCloud<Point>(elevated_cloud_ptr_,"no_ground_cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"ground_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"no_ground_cloud");

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void GroundRemoval::saveToPCD(std::string ground_cloud_file, std::string no_ground_cloud_file)
{
    pcl::PCDWriter writer;
    writer.write<Point>(ground_cloud_file, *ground_cloud_ptr_, true);
    writer.write<Point>(no_ground_cloud_file, *elevated_cloud_ptr_, true);
}
