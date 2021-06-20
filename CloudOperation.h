#include <pcl/common/io.h>
#include <pcl/point_types.h>

#ifndef CLOUD_OPERATION_HEADER
#define CLOUD_OPERATION_HEADER

class CloudOperation
{
public:
    CloudOperation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    virtual void processCloud() = 0;

protected:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;
};

#endif