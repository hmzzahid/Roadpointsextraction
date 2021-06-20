#include "CloudOperation.h"
#include <vector>

#ifndef CLOUD_CLUSTERING_HEADER
#define CLOUD_CLUSTERING_HEADER

class CloudClustering : CloudOperation
{
public:
    CloudClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    virtual void processCloud() override;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> getClusters() const;

    static const float TOLERANCE;
    static const int MIN;
    static const int MAX;

private:
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_clusters;
};

#endif