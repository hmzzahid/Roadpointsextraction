#include "CloudClustering.h"
#include <pcl/segmentation/extract_clusters.h>

const float CloudClustering::TOLERANCE = 0.2;
const int   CloudClustering::MIN = 10;
const int   CloudClustering::MAX = 750;

CloudClustering::CloudClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) : 
    CloudOperation(cloud)
{
}

void CloudClustering::processCloud() 
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(m_cloud);

    std::vector<pcl::PointIndices> e_clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(TOLERANCE); 
    ec.setMinClusterSize(MIN);
    ec.setMaxClusterSize(MAX);
    ec.setSearchMethod(tree);
    ec.setInputCloud(m_cloud);
    ec.extract(e_clusters);

    for (auto it = e_clusters.begin(); it != e_clusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*m_cloud)[*pit]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        m_clusters.push_back(cloud_cluster);
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> CloudClustering::getClusters() const
{
    return m_clusters;
}