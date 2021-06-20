#include "CloudOperation.h"

CloudOperation::CloudOperation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) : 
    m_cloud(cloud)
{
}