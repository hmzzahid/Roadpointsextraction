#include "CloudSegmentation.h"
#include <unordered_set>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>

CloudSegmentation::CloudSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) : 
    CloudOperation(cloud)
{
}

void CloudSegmentation::processCloud() 
{
    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(m_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(m_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize(20);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(m_cloud);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(2.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
        
    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    m_segmented_cloud = reg.getColoredCloud();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudSegmentation::getSegmentedCloud() const
{
    return m_segmented_cloud;
}
