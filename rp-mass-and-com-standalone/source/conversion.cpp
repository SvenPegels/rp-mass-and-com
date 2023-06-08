#include <pcl/point_types.h>


void pointXYZtoVector3f(pcl::PointXYZ &p_in, Eigen::Vector3f &p_out) {
    p_out = Eigen::Vector3f(p_in.x, p_in.y, p_in.z);
}

Eigen::Vector3f pointXYZtoVector3f(pcl::PointXYZ &p_in) {
    return Eigen::Vector3f(p_in.x, p_in.y, p_in.z);   
}


void vector3fToPointXYZ(Eigen::Vector3f &p_in, pcl::PointXYZ &p_out) {
    p_out = pcl::PointXYZ(p_in.x(), p_in.y(), p_in.z());
}

pcl::PointXYZ vector3fToPointXYZ(Eigen::Vector3f &p_in) {
    return pcl::PointXYZ(p_in.x(), p_in.y(), p_in.z());   
}