#include "CloudOperation.h"

#ifndef CLOUD_FILTERING_HEADER
#define CLOUD_FILTERING_HEADER

class CloudFiltering : CloudOperation
{
public:
    CloudFiltering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    virtual void processCloud() override;

    pcl::PointCloud<pcl::PointXYZI>::Ptr getFilteredCloud() const;

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_filtered_cloud;
};

#endif