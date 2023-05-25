/**
 * Volume estimation from .obj files
 * 
 * For code consistency:
 * - Visualizers as ::Ptr
 * - PointClouds/Meshes/etc as ::Ptr
 * - Single points not as ::Ptr. Passed as &reference to functions where necessary
*/

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_types.h>

#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// Surface estimation
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

// Surface normal estimation
#include <pcl/features/from_meshes.h>

/*Own code*/
#include "data_reading.cpp"
#include "visualization.cpp"
#include "bounding_box.cpp"
#include "volume_estimation.cpp"
#include "surface_utils.cpp"

#define OUTPUT_DIR ((std::string)"output/")

using namespace std::chrono_literals;

void meshFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PolygonMesh::Ptr mesh_res_ptr);
pcl::PolygonMesh meshFromPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
pcl::PolygonMesh::Ptr meshFromPointCloud3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
void dp(int n);
void initVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer_ptr);


/*
./volume_estimation_obj ~/rp-mass-and-com/pcd_files/rotated_torus.obj
*/
/**
 * Usage: ./volume_estimation_obj <.obj file>
 * Supports .obj files
*/
int main (int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "No .obj file argument given" << std::endl;
        return -1;
    }


    /*
    Loading file data
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ()); // TODO: Temp
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);
    // Load cloud data
    // TODO: Extract point cloud from mesh instead
    if (pcl::io::loadOBJFile(argv[1], *cloud_ptr) == 0) {
        std::cerr << ".obj file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud data from given .obj file" << std::endl;
        return -1;
    }
    // Load cloud and normal data
    if (pcl::io::loadOBJFile(argv[1], *cloud_w_normals_ptr) == 0) {
        std::cerr << ".obj file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud and normal data from given .obj file" << std::endl;
        return -1;
    }
    // Load mesh data
    if (pcl::io::loadOBJFile(argv[1], *mesh_ptr) == 0) {
        std::cerr << ".obj file found" << std::endl;
    } else {
        std::cerr << "Could not load mesh data from given .obj file" << std::endl;
        return -1;
    }
    pcl::fromPCLPointCloud2(mesh_ptr->cloud, *cloud_ptr);
    // pcl::fromPCLPointCloud2(mesh_ptr->cloud, *cloud_w_normals_ptr);
    // std::cerr << "---------------->" << std::endl;
    // std::cerr << "cloud size: '" << cloud_ptr->size() << "'" << std::endl;
    // std::cerr << "cloud w normals size: '" << cloud_w_normals_ptr->size() << "'" << std::endl;
    // std::cerr << "cloud size element: '" << cloud_ptr->at(0) << "'" << std::endl;
    // std::cerr << "cloud w normals element: '" << cloud_w_normals_ptr->at(0) << "'" << std::endl;
    // std::cerr << "cloud w normals size: '" << cloud_w_normals_ptr->size() << "'" << std::endl;
    // std::cerr << "cloud size element: '" << cloud_ptr->at(99) << "'" << std::endl;
    // std::cerr << "cloud w normals element: '" << cloud_w_normals_ptr->at(99) << "'" << std::endl;
    // std::cerr << "---------------->" << std::endl;


    /*
    Visualization setup
    */
    pcl::visualization::PCLVisualizer::Ptr viewer_ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    initVisualizer(viewer_ptr);
    viewer_ptr->addPolygonMesh(*mesh_ptr, "polygon mesh");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "polygon mesh");
    

    /*
    AABB and OBB
    */
    // Get bounding box data
    pcl::PointXYZ AABB_min_point;
    pcl::PointXYZ AABB_max_point;
    pcl::PointXYZ OBB_min_point;
    pcl::PointXYZ OBB_max_point;
    pcl::PointXYZ OBB_pos;
    Eigen::Matrix3f OBB_rot;
    getBoundingBoxes(cloud_ptr, AABB_min_point, AABB_max_point, OBB_min_point, OBB_max_point, OBB_pos, OBB_rot);
    std::cerr << "AABB_min_point: '" << AABB_min_point << "'" << std::endl;

    // Draw AABB in blue
    drawBoundingBox(viewer_ptr, AABB_min_point, AABB_max_point, 0.0, 0.0, 1.0, "AABB");
    // Draw OBB in red
    drawBoundingBox(viewer_ptr, OBB_min_point, OBB_max_point, OBB_pos, OBB_rot, 1.0, 0.0, 0.0, "OBB");

    // Read actual size (assuming file only contains volume data)
    float vol_actual = readActualVolume(argv[1]);
    
    // Calculate size
    float vol_AABB = calcBoxVolume(AABB_min_point, AABB_max_point);
    float vol_OBB = calcBoxVolume(OBB_min_point, OBB_max_point);
    

    /*
    Estimate volume using tetrahedrons
    */
    std::cerr << "Mesh polygons size: '" << mesh_ptr->polygons.size() << "'" << std::endl;

    // Estimate surface normals
    // pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals_ptr (new pcl::PointCloud<pcl::PointNormal>());
    // pcl::features::computeApproximateNormals(*cloud_ptr, mesh_ptr->polygons, *surface_normals_ptr);
    // pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals_at_center_ptr (new pcl::PointCloud<pcl::PointNormal>());
    // pcl::features::computeApproximateNormals(*cloud_ptr, mesh_ptr->polygons, *surface_normals_at_center_ptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals_ptr (new pcl::PointCloud<pcl::PointNormal>());

    // TODO: Fix and add to thesis: Produces problem with vertices being oriented towards the viewpoint (as described in bachelors thesis), where viewpoint is assumed to be at (0,0,0)
    pcl::features::computeApproximateNormals(*cloud_ptr, mesh_ptr->polygons, *surface_normals_ptr);
    // surface normals are added to the .getNormalVector3fMap()
    // I think this calculates the surface normal by



    // Calculate surface centers (centroids)
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_centroids_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    calcSurfaceCentroids(mesh_ptr, cloud_ptr, surface_centroids_ptr);

    std::cerr << std::endl;

    // Estimate centroid normals
    // Centroid normal is estimated using the cross product of the vectors from p1 to p2, and p1 to p3.
    pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr (new pcl::PointCloud<pcl::Normal>());
    estimateSurfaceNormals(mesh_ptr, cloud_w_normals_ptr, centroid_normals_ptr);
    std::cerr << std::endl;


    pcl::PointCloud<pcl::PointNormal>::Ptr surface_centroids_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*surface_centroids_ptr, *centroid_normals_ptr, *surface_centroids_w_normals_ptr);

    // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(surface_centroids_ptr, centroid_normals_ptr, 1, 0.05f, "centroid normals");
    viewer_ptr->addPointCloudNormals<pcl::PointNormal>(surface_centroids_w_normals_ptr, 1, 0.05f, "centroid normals");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.0, 0.0, "centroid normals");






     

    // Display normals at the surface center
    // std::cerr << "before adding surface normals" << std::endl;
    // std::cerr << "n_mesh_polygons : '" << mesh_ptr->polygons.size() << "'" << std::endl;
    // std::cerr << "n_surface_centers : '" << surface_centroids_ptr->size() << "'" << std::endl;
    // std::cerr << "n_surface_normals : '" << surface_normals_ptr->size() << "'" << std::endl;
    // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(surface_centroids_ptr, surface_normals_ptr, 1, 0.05f, "surface normals"); //TODO: Add normal points
    // // viewer->addPointCloudNormals<pcl::PointNormal>(surface_normals_ptr, 5, 0.05f, "surface normals 2");

    // std::cerr << "after adding surface normals" << std::endl;
    

    // Calculate volume
    pcl::index_t offset = 0;
    float vol_tetra = calcMeshVolumeTetrahedron(mesh_ptr, cloud_ptr, centroid_normals_ptr);
    std::cerr << "Total tetra vol: '" << vol_tetra << "'" << std::endl;

    


    // Display size
    displayText(viewer_ptr, "Actual volume: " + std::to_string(vol_actual));
    displayText(viewer_ptr, "AABB volume: " + std::to_string(vol_AABB));
    displayText(viewer_ptr, "OBB volume: " + std::to_string(vol_OBB));
    displayText(viewer_ptr, "Tetra volume: " + std::to_string(vol_tetra));




    // {
    //     /*
    //     Greedy triangulation
    //     */
    //     pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);
    //     pcl::PolygonMesh mesh;
    //     dp(0);
    //     meshFromPointCloud(cloud_ptr, mesh_ptr);
    //     std::cerr << "Resulting cloud param: " << mesh_ptr->cloud.height << "x" << mesh_ptr->cloud.width << std::endl;
    //     mesh = meshFromPointCloud2(cloud_ptr);
    //     std::cerr << "Resulting cloud param: " << mesh.cloud.height << "x" << mesh.cloud.width << std::endl;
    //         // Estimate normals
        // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
    //     pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ptr (new pcl::search::KdTree<pcl::PointXYZ>);
    //     tree_ptr->setInputCloud (cloud_ptr);
    //     normal_est.setInputCloud (cloud_ptr);
    //     normal_est.setSearchMethod (tree_ptr);
    //     normal_est.setKSearch (20);
    //     normal_est.compute (*normals_ptr);
    //     dp(1);

    //     // viewer->addPointCloudNormals(normals_ptr);

    //     // Concatenate the XYZ and normal fields*
    //     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    //     pcl::concatenateFields (*cloud_ptr, *normals_ptr, *cloud_with_normals);
    //     dp(2);

    //     // viewer->addPointCloudNormals(cloud_with_normals);
    //     viewer->addPointCloudNormals<pcl::PointNormal>(cloud_with_normals, 1, 0.05f, "cloud with normals");

    //     // Create search tree
    //     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    //     tree2->setInputCloud (cloud_with_normals);
    //     dp(3);

    //     // Initialize objects
    //     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //     // pcl::PolygonMesh triangles;

    //     // Set the maximum distance between connected points (maximum edge length)
    //     gp3.setSearchRadius (0.025);

    //     // Set typical values for the parameters
    //     gp3.setMu (2.5);
    //     gp3.setMaximumNearestNeighbors (100);
    //     gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    //     gp3.setMinimumAngle(M_PI/18); // 10 degrees
    //     gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    //     gp3.setNormalConsistency(false);
    //     dp(40);

    //     // Get result
    //     gp3.setInputCloud (cloud_with_normals);
    //     gp3.setSearchMethod (tree2);
    //     dp(41);
    //     // gp3.reconstruct (*mesh_ptr);
    //     gp3.reconstruct(mesh);
    //     dp(5);

    //     // Additional vertex information
    //     std::vector<int> parts = gp3.getPartIDs();
    //     dp(6);
    //     std::vector<int> states = gp3.getPointStates();
    //     dp(7);

    //     std::cerr << "Resulting cloud param: " << mesh_ptr->cloud.height << "x" << mesh_ptr->cloud.width << std::endl;
    //     // std::cerr << "Resulting cloud local: " << triangles.cloud.height << "x" << triangles.cloud.width << std::endl;
    //     dp(8);

    //     // Display mesh
    //     viewer->addPolygonMesh(*mesh_ptr);
    //     viewer->addPolygonMesh(mesh);
    
    // }
    // dp(00);
    

    // Run until window is closed
    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}



/**
 * Creates a mesh from the given point cloud using a Greedy Triangulation Algorithm
 * Taken from: https://pcl.readthedocs.io/projects/tutorials/en/latest/greedy_projection.html
 * See refernce for more info:
  @InProceedings{Marton09ICRA,
  author    = {Zoltan Csaba Marton and Radu Bogdan Rusu and Michael Beetz},
  title     = {{On Fast Surface Reconstruction Methods for Large and Noisy Datasets}},
  booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)},
  month     = {May 12-17},
  year      = {2009},
  address   = {Kobe, Japan},
    }
*/
void meshFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PolygonMesh::Ptr mesh_res_ptr) {
    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ptr (new pcl::search::KdTree<pcl::PointXYZ>);
    tree_ptr->setInputCloud (cloud_ptr);
    normal_est.setInputCloud (cloud_ptr);
    normal_est.setSearchMethod (tree_ptr);
    normal_est.setKSearch (20);
    normal_est.compute (*normals_ptr);
    dp(1);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_ptr, *normals_ptr, *cloud_with_normals);
    dp(2);

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    dp(3);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    // pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    dp(40);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    dp(41);
    gp3.reconstruct (*mesh_res_ptr);
    // gp3.reconstruct(triangles);
    dp(5);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    dp(6);
    std::vector<int> states = gp3.getPointStates();
    dp(7);

    std::cerr << "Resulting cloud param: " << mesh_res_ptr->cloud.height << "x" << mesh_res_ptr->cloud.width << std::endl;
    // std::cerr << "Resulting cloud local: " << triangles.cloud.height << "x" << triangles.cloud.width << std::endl;
    dp(8);

}

void dp(int n) {
    std::cerr << "Here " << n << std::endl;
}

/*
Initializes PCLVisualizer with preferred values
*/
void initVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer_ptr) {
    viewer_ptr->setBackgroundColor (0, 0, 0);
    viewer_ptr->addCoordinateSystem (1.0);
    viewer_ptr->initCameraParameters ();
    viewer_ptr->setCameraPosition(-5,5,-5, 0,0,0);
    // Add center (0,0,0) sphere
    viewer_ptr->addSphere((pcl::PointXYZ(0.0, 0.0, 0.0)), 0.05, "sphere", 0);
}