#include "CloudOperation.h"

#ifndef CLOUD_SEGMENTATION_HEADER
#define CLOUD_SEGMENTATION_HEADER

class CloudSegmentation : CloudOperation
{
public:
    CloudSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    virtual void processCloud() override;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSegmentedCloud() const;

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmented_cloud;
};

#endif