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
#include <vector>

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
    Loading 3d data
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

    // Read actual size (assuming file only contains volume data)
    float vol_actual = readActualVolume(argv[1]);


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

    // Draw AABB in blue
    drawBoundingBox(viewer_ptr, AABB_min_point, AABB_max_point, 0.0, 0.0, 1.0, "AABB");
    // Draw OBB in red
    drawBoundingBox(viewer_ptr, OBB_min_point, OBB_max_point, OBB_pos, OBB_rot, 1.0, 0.0, 0.0, "OBB");
    
    // Calculate size
    float vol_AABB = calcBoxVolume(AABB_min_point, AABB_max_point);
    float vol_OBB = calcBoxVolume(OBB_min_point, OBB_max_point);
    

    /*
    Volume estimation using tetrahedrons
    */
    std::cerr << "Mesh polygons size: '" << mesh_ptr->polygons.size() << "'" << std::endl;

    // Estimate surface normals

    // pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals_ptr (new pcl::PointCloud<pcl::PointNormal>());

    // TODO: Fix and add to thesis: Produces problem with vertices being oriented towards the viewpoint (as described in bachelors thesis), where viewpoint is assumed to be at (0,0,0)
    // pcl::features::computeApproximateNormals(*cloud_ptr, mesh_ptr->polygons, *surface_normals_ptr);
    // surface normals are added to the .getNormalVector3fMap()
    // I think this calculates the surface normal by

    // Calculate surface centers (centroids)
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_centroids_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    calcSurfaceCentroids(mesh_ptr, cloud_ptr, surface_centroids_ptr);

    // Estimate centroid normals
    // Centroid normal is estimated using the cross product of the vectors from p1 to p2, and p1 to p3.
    pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr (new pcl::PointCloud<pcl::Normal>());
    estimateSurfaceNormals(mesh_ptr, cloud_w_normals_ptr, centroid_normals_ptr);
    
    // Combine centroid xyz and normal
    pcl::PointCloud<pcl::PointNormal>::Ptr surface_centroids_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*surface_centroids_ptr, *centroid_normals_ptr, *surface_centroids_w_normals_ptr);

    // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(surface_centroids_ptr, centroid_normals_ptr, 1, 0.05f, "centroid normals");
    viewer_ptr->addPointCloudNormals<pcl::PointNormal>(surface_centroids_w_normals_ptr, 1, 0.05f, "centroid normals");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.0, 0.0, "centroid normals");

    // Calculate volume
    pcl::index_t offset = 0;
    float vol_tetra = calcMeshVolumeTetrahedron(mesh_ptr, cloud_ptr, centroid_normals_ptr);
    std::cerr << "Total tetra vol: '" << vol_tetra << "'" << std::endl;


    /*
    Convex Hull
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    calcConvexHull3D(cloud_ptr, convex_hull_cloud_ptr);

    pcl::PolygonMesh::Ptr convex_hull_mesh_ptr (new pcl::PolygonMesh);
    double vol_chull = calcConvexHull3D(cloud_ptr, convex_hull_mesh_ptr);

    // Visualize hull points
    viewer_ptr->addPointCloud(convex_hull_cloud_ptr, "convex hull cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "convex hull cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "convex hull cloud");

    // Visualize hull mesh
    viewer_ptr->addPolygonMesh(*convex_hull_mesh_ptr, "convex hull mesh");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.5, "convex hull mesh");
    // viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "convex hull mesh");


    /*
    Display results
    */
    // Display sizes in the bottom left corner of the screen
    displayText(viewer_ptr, "Actual volume: " + std::to_string(vol_actual));
    displayText(viewer_ptr, "AABB volume: " + std::to_string(vol_AABB));
    displayText(viewer_ptr, "OBB volume: " + std::to_string(vol_OBB));
    displayText(viewer_ptr, "Tetra volume: " + std::to_string(vol_tetra));
    displayText(viewer_ptr, "Convex Hull volume: " + std::to_string(vol_chull));
    

    // Run until window is closed
    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
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