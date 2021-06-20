#include "CloudFiltering.h"
#include <pcl/filters/voxel_grid.h>

CloudFiltering::CloudFiltering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) : 
    CloudOperation(cloud),
    m_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>())
{
}

void CloudFiltering::processCloud() 
{
    pcl::VoxelGrid<pcl::PointXYZI> vox;
    vox.setInputCloud(m_cloud);
    vox.setLeafSize(0.15f, 0.15f, 0.15f);
    vox.filter(*m_filtered_cloud);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudFiltering::getFilteredCloud() const
{
    return m_filtered_cloud;
}