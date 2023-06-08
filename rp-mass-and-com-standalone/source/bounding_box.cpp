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


/*
Rotates the given 'point_in' by 'rotation', then translates it by 'position', returning the result in 'point_out'*/
void getOBBPoint(pcl::PointXYZ &point_in, Eigen::Matrix3f rotation, pcl::PointXYZ &position, pcl::PointXYZ &point_out) {
    Eigen::Vector3f point_vectro = Eigen::Vector3f(point_in.x, point_in.y, point_in.z);
    Eigen::Vector3f point_rotated = rotation * point_vectro;
    Eigen::Vector3f position_vector = Eigen::Vector3f(position.x, position.y, position.z);
    Eigen::Vector3f point_translated = position_vector + point_rotated;
    point_out = pcl::PointXYZ(point_translated.x(), point_translated.y(), point_translated.z());
}