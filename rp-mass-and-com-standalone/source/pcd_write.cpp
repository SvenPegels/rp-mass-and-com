#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h> // For AABB, OBB
#include <time.h>

#define OUTPUT_DIR ((std::string)"output/")

int main () {
    srand((unsigned int)time(0));
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);

    for (auto& point: cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII (OUTPUT_DIR + "test_pcd.pcd", cloud);

    std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;
    
    for (const auto& point: cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
}

/*
Calculate Axis Aligned Bounding Box (AABB)
*/
void calcAABBfromCloud(pcl::PointCloud<pcl::PointXYZ> cloud) {
    // pcl::MomentOfInertiaEstimation<PointXYZ> moie;
}

/*
Estimate Oriented Bounding Box (OBB)
*/
void estimateOBBromCloud(pcl::PointCloud<pcl::PointXYZ> cloud) {

}