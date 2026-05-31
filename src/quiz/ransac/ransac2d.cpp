/* \author Aaron Brown */

// RANSAC plane fitting in 3D

#include "../../render/render.h"
#include <unordered_set>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers (roughly y = x)
    float scatter = 0.6f;
    for (int i = -5; i < 5; i++)
    {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;
        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--)
    {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

// Return indices of inliers for best-fit PLANE using RANSAC in 3D
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations,
                                    float distanceTol)
{
    std::unordered_set<int> bestInliers;
    if (!cloud || cloud->points.size() < 3) return bestInliers;

    srand((unsigned)time(nullptr));
    const int N = static_cast<int>(cloud->points.size());

    auto t0 = std::chrono::steady_clock::now();

    for (int it = 0; it < maxIterations; ++it)
    {
        // 1) Randomly sample 3 distinct points
        int i1 = rand() % N;
        int i2 = rand() % N;
        int i3 = rand() % N;
        if (i1 == i2 || i1 == i3 || i2 == i3) { --it; continue; }

        const auto& p1 = cloud->points[i1];
        const auto& p2 = cloud->points[i2];
        const auto& p3 = cloud->points[i3];

        // 2) Compute plane coefficients from 3 points
        // v1 = p2 - p1, v2 = p3 - p1
        float v1x = p2.x - p1.x, v1y = p2.y - p1.y, v1z = p2.z - p1.z;
        float v2x = p3.x - p1.x, v2y = p3.y - p1.y, v2z = p3.z - p1.z;

        // normal = v1 x v2 = (A,B,C)
        float A = v1y * v2z - v1z * v2y;
        float B = v1z * v2x - v1x * v2z;
        float C = v1x * v2y - v1y * v2x;

        // Degenerate if the three points are colinear (||normal|| == 0)
        float norm = std::sqrt(A*A + B*B + C*C);
        if (norm == 0.0f) { --it; continue; }

        // D = -(A*x1 + B*y1 + C*z1)
        float D = -(A * p1.x + B * p1.y + C * p1.z);

        // 3) Score inliers by pointplane distance
        std::unordered_set<int> inliers;
        inliers.reserve(N);
        for (int idx = 0; idx < N; ++idx)
        {
            const auto& p = cloud->points[idx];
            float dist = std::fabs(A * p.x + B * p.y + C * p.z + D) / norm;
            if (dist <= distanceTol)
                inliers.insert(idx);
        }

        if (inliers.size() > bestInliers.size())
            bestInliers.swap(inliers);
    }

    auto t1 = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "RANSAC plane took " << ms << " ms. Inliers: " << bestInliers.size() << std::endl;

    return bestInliers;
}

int main()
{
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data (3D highway point cloud)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    // RANSAC parameters for plane
    int   maxIterations = 100;
    float distanceTol   = 0.2f;

    std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceTol);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < static_cast<int>(cloud->points.size()); index++)
    {
        const auto& point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Render
    if (inliers.size())
    {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    }
    else
    {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}