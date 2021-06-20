// OOP
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

#include "CloudClustering.h"
#include "CloudFiltering.h"
#include "CloudRansac.h"
#include "CloudSegmentation.h"
#include "CloudVisualization.h"


int main(int argc, char** argv)
{
    // Load new point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("Data/000000.pcd", *source_cloud) == -1) 
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    // Filtering
    auto filter = new CloudFiltering(source_cloud);
    filter->processCloud();

    // Ransac
    auto ransac = new CloudRansac(filter->getFilteredCloud());
    ransac->processCloud();

    // Segmentation
    auto segmentation = new CloudSegmentation(ransac->getInliers());
    segmentation->processCloud();

    // Clustering
    auto clustering = new CloudClustering(ransac->getOutliers());
    clustering->processCloud();

    // Visualization
    auto visualizer = new CloudVisualization();
    visualizer->addRoad(segmentation->getSegmentedCloud());
    visualizer->addClusters(clustering->getClusters());
    visualizer->visualize();

    // Cleanup
    delete visualizer;
    delete clustering;
    delete segmentation;
    delete ransac;
    delete filter;
}
