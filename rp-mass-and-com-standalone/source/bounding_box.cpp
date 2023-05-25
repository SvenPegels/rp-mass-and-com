/**
 * Everything to do with AABB and OBB bounding boxes
*/

// #include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>


//TODO: Improve documentation
/*
Compute Bounding Box features.
Computes AABB and OBB features.
*/
void getBoundingBoxes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, 
        pcl::PointXYZ &AABB_min_point_out, pcl::PointXYZ &AABB_max_point_out, 
        pcl::PointXYZ &OBB_min_point, pcl::PointXYZ &OBB_max_point_out, pcl::PointXYZ &OBB_pos, Eigen::Matrix3f &OBB_rot_out
        ) {
    // Compute features
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_ptr);
    feature_extractor.compute();

    // Extract features
    feature_extractor.getAABB(AABB_min_point_out, AABB_max_point_out);
    feature_extractor.getOBB(OBB_min_point, OBB_max_point_out, OBB_pos, OBB_rot_out);
}