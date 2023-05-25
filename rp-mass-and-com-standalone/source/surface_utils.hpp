
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>

/*
Calculates surface centroids based on polygon vertices.
Assumes all surface polygons are triangles.
*/
void calcSurfaceCentroids(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr surface_centroids_ptr_out);

/*
Estimates the surface centroid normals based on the polygon vertices.
Assumes all surface polygons are triangles (have 3 vertices).
*/
void estimateSurfaceNormals(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr, pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr_out, bool correct_normals);

/*
Flips the normals 180 degrees
*/
void flipNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_ptr);

/*
Flips the normal 180 degrees
*/
void flipNormal(Eigen::Vector3f &normal);
