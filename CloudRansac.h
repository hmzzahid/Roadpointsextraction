#include "CloudOperation.h"

#ifndef CLOUD_RANSAC_HEADER
#define CLOUD_RANSAC_HEADER

class CloudRansac : CloudOperation
{
public:
    CloudRansac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    virtual void processCloud() override;

    pcl::PointCloud<pcl::PointXYZI>::Ptr getInliers() const;
    pcl::PointCloud<pcl::PointXYZI>::Ptr getOutliers() const;

    static const int MAX_ITERATIONS;
    static const float DISTANCE;

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_inliers;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_outliers;
};

#endif