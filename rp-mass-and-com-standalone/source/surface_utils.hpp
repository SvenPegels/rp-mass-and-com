
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>

// Convex hull
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>


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


/*
Calculates a convex hull around the given 3D PointCloud.
Outputs the vertices of the hull.
*/
void calcConvexHull3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_cloud_ptr_out);


/*
Calculates a convex hull around the given 3D PointCloud.
Outputs the mesh of the hull/
*/
double calcConvexHull3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PolygonMesh::Ptr convex_hull_mesh_ptr_out);


/*
Calculates a 2D convex hull around some projection of the given PointCloud.
THIS IS FOR 2D.
*/
void calcConvexHull2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_ptr_out);