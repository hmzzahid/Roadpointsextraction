#include <vector>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#ifndef CLOUD_VISUALIZATION_HEADER
#define CLOUD_VISUALIZATION_HEADER

class CloudVisualization
{
public:
    CloudVisualization();

    void addRoad(const pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);
    void addClusters(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters);

    void visualize() const;

protected:
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
};

#endif