
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>


void calcSurfaceCentroids(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr surface_centroids_ptr_out);
void estimateSurfaceNormals(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr, pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr_out);

void flipNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_ptr);
void flipNormal(Eigen::Vector3f &normal);
