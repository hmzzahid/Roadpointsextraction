#include "CloudVisualization.h"
#include <string>
#include <pcl/common/common.h>

CloudVisualization::CloudVisualization() :
    m_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
{
    m_viewer->setBackgroundColor(0, 0, 0);
    m_viewer->initCameraParameters();
    m_viewer->setCameraPosition(-20, -20, 20, 1, 1, 0);;
    m_viewer->addCoordinateSystem(1.0);
}

void CloudVisualization::addRoad(const pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud)
{
    m_viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "road");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "road");
    //m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "road");
}

void CloudVisualization::addClusters(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters)
{
    float x_min, y_min, z_min, x_max, y_max, z_max;

    for (int i = 0; i < clusters.size(); i++)
    {
        if (clusters.at(i)->size() < 30)
            continue;

        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(*(clusters.at(i)), minPoint, maxPoint);

        x_min = minPoint.x;
        x_max = maxPoint.x;
        y_min = minPoint.y;
        y_max = maxPoint.y;
        z_min = minPoint.z;
        z_max = maxPoint.z;

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(clusters.at(i), "intensity");
        m_viewer->addPointCloud<pcl::PointXYZI>(clusters.at(i), intensity_distribution, "clusters" + std::to_string(i));
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "clusters" + std::to_string(i));

        m_viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0, 0, 1, "bb" + std::to_string(i));
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bb" + std::to_string(i));
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "bb" + std::to_string(i));
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "bb" + std::to_string(i));
    }
}

void CloudVisualization::visualize() const
{
    while (!m_viewer->wasStopped()) {
        m_viewer->spinOnce();
    }
}