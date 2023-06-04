/**
 * Volume estimation from .pcd files
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

#include <pcl/common/transforms.h>

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
./visualize_partial_view ~/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/torus_triangulated_lc.obj ~/rp-mass-and-com/pcd_files/partial_view_generated_clouds/torus_triangulated_lc_cam_2.pcd ~/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/data/torus_triangulated_lc_data.txt
*/
/**
 * Usage: ./volume_estimation_pcd <.pcd file> (<data file>)
 * Supports .pcd files
*/
int main (int argc, char** argv) {
    /*
    Check arguments and retrieve data file path
    */
    std::string data_file_path;
    if (argc != 4) {
        std::cerr << "Not enough arguments! Give the original .obj, the generated .pcd and the _data.txt" << std::endl;
        return -1;
    }
        data_file_path = argv[3];


    /*
    Loading 3d data
    */
    pcl::PolygonMesh::Ptr original_mesh_ptr (new pcl::PolygonMesh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr partial_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    // Load partial view cloud data
    if (pcl::io::loadPCDFile(argv[2], *partial_cloud_ptr) == 0) {
        std::cerr << ".pcd file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud data from given .pcd file" << std::endl;
        return -1;
    }
    // Load original mesh
    if (pcl::io::loadOBJFile(argv[1], *original_mesh_ptr) == 0) {
        std::cerr << ".pcd file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud and normal data from given .pcd file" << std::endl;
        return -1;
    }
    // Load original point cloud
    if (pcl::io::loadOBJFile(argv[1], *original_cloud_ptr) == 0) {
        std::cerr << ".pcd file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud and normal data from given .pcd file" << std::endl;
        return -1;
    }

    // Read actual size (assuming file only contains volume data)
    float vol_actual = readActualVolumeFromDataFile(data_file_path);


    /*
    Visualization setup
    */
    pcl::visualization::PCLVisualizer::Ptr viewer_ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    initVisualizer(viewer_ptr);


    /*
    Visualize point cloud
    */
    viewer_ptr->addPointCloud(partial_cloud_ptr, "cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");


    /*
    Visualise original mesh, translated by 30 over the x-axis
    */
    Eigen::Affine3f transform_og = Eigen::Affine3f::Identity();
    transform_og.translation() << 30, 0.0, 0.0; //TODO: Take a number dynamically instead of assuming objects are a certain size

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_original_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*original_cloud_ptr, *transformed_original_cloud_ptr, transform_og);
    viewer_ptr->addPolygonMesh<pcl::PointXYZ>(transformed_original_cloud_ptr, original_mesh_ptr->polygons, "original mesh");


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
    getBoundingBoxes(partial_cloud_ptr, AABB_min_point, AABB_max_point, OBB_min_point, OBB_max_point, OBB_pos, OBB_rot);

    // // Draw AABB in blue
    // drawBoundingBox(viewer_ptr, AABB_min_point, AABB_max_point, 0.0, 0.0, 1.0, "AABB");
    // // Draw OBB in red
    // drawBoundingBox(viewer_ptr, OBB_min_point, OBB_max_point, OBB_pos, OBB_rot, 1.0, 0.0, 0.0, "OBB");
    
    // Calculate size
    float vol_AABB = calcBoxVolume(AABB_min_point, AABB_max_point);
    float vol_OBB = calcBoxVolume(OBB_min_point, OBB_max_point);

    // Calculate AABB center point (average of AABB min and max)
    pcl::PointXYZ AABB_center_point = pcl::PointXYZ((AABB_min_point.x+AABB_max_point.x)/2, (AABB_min_point.y+AABB_max_point.y)/2, (AABB_min_point.z+AABB_max_point.z)/2);
    

    /*
    Convex Hull
    */
    // Calc convex hull cloud and mesh
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    calcConvexHull3D(partial_cloud_ptr, convex_hull_cloud_ptr);

    pcl::PolygonMesh::Ptr convex_hull_mesh_ptr (new pcl::PolygonMesh);
    double vol_chull = calcConvexHull3D(partial_cloud_ptr, convex_hull_mesh_ptr);

    // Visualize hull cloud
    viewer_ptr->addPointCloud(convex_hull_cloud_ptr, "convex hull cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "convex hull cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "convex hull cloud");

    // Translate hull mesh by -30 over the x-axis
    Eigen::Affine3f transform_hull = Eigen::Affine3f::Identity();
    transform_hull.translation() << -30, 0.0, 0.0;

    // Visualize translated hull mesh
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_chull_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*convex_hull_cloud_ptr, *transformed_chull_cloud_ptr, transform_hull);
    viewer_ptr->addPolygonMesh<pcl::PointXYZ>(transformed_chull_cloud_ptr, convex_hull_mesh_ptr->polygons, "translated convex hull mesh");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.5, "translated convex hull mesh");


    /*
    Display results
    */
    // Display volumes in the bottom left corner of the screen
    displayText(viewer_ptr, "Actual volume: " + std::to_string(vol_actual));
    displayText(viewer_ptr, "AABB volume: " + std::to_string(vol_AABB));
    displayText(viewer_ptr, "OBB volume: " + std::to_string(vol_OBB));
    displayText(viewer_ptr, "Convex Hull volume: " + std::to_string(vol_chull));

    // Print volumes
    std::cerr << "Actual volume: '" << vol_actual << "'" << std::endl;
    std::cerr << "AABB volume: '" << vol_AABB << "'" << std::endl;
    std::cerr << "OBB volume: '" << vol_OBB << "'" << std::endl;
    std::cerr << "Convex Hull volume: '" << vol_chull << "'" << std::endl;
    

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
    // viewer_ptr->setBackgroundColor (1, 1, 1);
    viewer_ptr->setBackgroundColor (0, 0, 0);
    viewer_ptr->addCoordinateSystem (1.0);
    viewer_ptr->initCameraParameters ();
    // viewer_ptr->setcamera
    viewer_ptr->setCameraPosition(25.5879, 22.7587, -70.9508, -0.230643, 0.947866, 0.219896);
    // Add center (0,0,0) sphere
    viewer_ptr->addSphere((pcl::PointXYZ(0.0, 0.0, 0.0)), 0.05, "sphere", 0);
}