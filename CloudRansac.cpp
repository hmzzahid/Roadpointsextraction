#include "CloudRansac.h"
#include <unordered_set>

const int   CloudRansac::MAX_ITERATIONS = 50;
const float CloudRansac::DISTANCE = 0.25;


CloudRansac::CloudRansac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) : 
    CloudOperation(cloud),
    m_inliers(new pcl::PointCloud<pcl::PointXYZI>()),
    m_outliers(new pcl::PointCloud<pcl::PointXYZI>())
{
}

void CloudRansac::processCloud() 
{
    std::unordered_set<int> inliersRes;
    constexpr int k = 3;
    float a, b, c, d, len;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        std::unordered_set<int> inliers;

        float x[k], y[k], z[k];
        int j = 0;

        // Avoiding of picking the same point two times 
        while (inliers.size() < k) {
            auto index = rand() % (m_cloud->points.size());
            x[j] = m_cloud->points[index].x;
            y[j] = m_cloud->points[index].y;
            z[j] = m_cloud->points[index].z;

            if (0 == inliers.count(index)) {
                inliers.insert(index);
                j++;
            }
        }

        a = ((y[1] - y[0]) * (z[2] - z[0])) - ((z[1] - z[0]) * (y[2] - y[0]));
        b = ((z[1] - z[0]) * (x[2] - x[0])) - ((x[1] - x[0]) * (z[2] - z[0]));
        c = ((x[1] - x[0]) * (y[2] - y[0])) - ((y[1] - y[0]) * (x[2] - x[0]));
        d = -(a * x[0] + b * y[0] + c * z[0]);

        len = sqrt(a * a + b * b + c * c);

        // Measure the distance between every point
        for (int i = 0; i < m_cloud->points.size(); i++) {
            if (inliers.count(i) > 0) {
                continue;
            }

            auto point = m_cloud->points[i];
            float x = point.x;
            float y = point.y;
            float z = point.z;

            float distance = fabs(a * x + b * y + c * z + d) / len;

            // Inlier - if the distance is smaller then the threshold
            if (distance <= DISTANCE) {
                inliers.insert(i);
            }

            if (inliers.size() > inliersRes.size()) {
                inliersRes = inliers;
            }

        }

    }

    for (int index = 0; index < m_cloud->points.size(); index++)
    {
        auto point = m_cloud->points[index];
        if (inliersRes.count(index))
            m_inliers->points.push_back(point);
        else
            m_outliers->points.push_back(point);
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRansac::getInliers() const
{
    return m_inliers;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRansac::getOutliers() const
{
    return m_outliers;
}