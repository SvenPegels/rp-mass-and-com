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
./volume_estimation_pcd ~/rp-mass-and-com/pcd_files/rotated_torus.pcd
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
    if (argc >= 3) {
        // .pcd and data.txt given
        data_file_path = argv[2];
    } else if (argc >= 2) {
        // .pcd or data.txt not given. Likely only .pcd
        std::cerr << "No .pcd or data file argument given. Assuming .pcd is given. Defaulting to _data.txt for data." << std::endl;
        if (dataFilePathFromObjectFilePath(argv[1], data_file_path) != 0) {
            data_file_path = "OBJECT_FILE_NOT_SUPPORTED";
            std::cerr << "Object file type not supported!" << std::endl;
            return -1;
        }
    } else {
        std::cerr << "No .pcd file argument given!" << std::endl;
        return -1;
    }


    /*
    Loading 3d data
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ()); // TODO: Temp
    // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal> ());
    // pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);
    // Load cloud data
    // TODO: Extract point cloud from mesh instead
    if (pcl::io::loadPCDFile(argv[1], *cloud_ptr) == 0) {
        std::cerr << ".pcd file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud data from given .pcd file" << std::endl;
        return -1;
    }
    // // Load cloud and normal data
    // if (pcl::io::loadPCDFile(argv[1], *cloud_w_normals_ptr) == 0) {
    //     std::cerr << ".pcd file found" << std::endl;
    // } else {
    //     std::cerr << "Could not load point cloud and normal data from given .pcd file" << std::endl;
    //     return -1;
    // }

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
    viewer_ptr->addPointCloud(cloud_ptr, "cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    

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

    // Calculate AABB center point (average of AABB min and max)
    pcl::PointXYZ AABB_center_point = pcl::PointXYZ((AABB_min_point.x+AABB_max_point.x)/2, (AABB_min_point.y+AABB_max_point.y)/2, (AABB_min_point.z+AABB_max_point.z)/2);
    

    /*
    Convex Hull
    //TODO: Clean up code
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    calcConvexHull3D(cloud_ptr, convex_hull_cloud_ptr);

    pcl::PolygonMesh::Ptr convex_hull_mesh_ptr (new pcl::PolygonMesh);
    double vol_chull = calcConvexHull3D(cloud_ptr, convex_hull_mesh_ptr);

    // Visualize hull points
    viewer_ptr->addPointCloud(convex_hull_cloud_ptr, "convex hull cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "convex hull cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "convex hull cloud");

    // // Create translated hull mesh to
    // pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_mesh_points_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(convex_hull_mesh_ptr->cloud, *convex_hull_mesh_points_ptr);
    // /*  METHOD #2: Using a Affine3f
    //  This method is easier and less error prone
    // */
    // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // // Define a translation of 2.5 meters on the x axis.
    // transform_2.translation() << 2.5, 0.0, 0.0;

    // // The same rotation matrix as before; theta radians around Z axis
    // // transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    // // Print the transformation
    // printf ("\nMethod #2: using an Affine3f\n");
    // std::cout << transform_2.matrix() << std::endl;

    // // Executing the transformation
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // // You can either apply transform_1 or transform_2; they are the same
    // pcl::transformPointCloud (*convex_hull_mesh_points_ptr, *transformed_cloud, transform_2);
    // // pcl::PointCloud<pcl::PointXYZ>::ConstPtr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ>(*transformed_cloud));
    // viewer_ptr->addPolygonMesh<pcl::PointXYZ>(transformed_cloud, convex_hull_mesh_ptr->polygons, "convex hull mesh translated");
    // // viewer_ptr->addPolygonMesh()

    // Visualize hull mesh
    viewer_ptr->addPolygonMesh(*convex_hull_mesh_ptr, "convex hull mesh");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.5, "convex hull mesh");
    // viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "convex hull mesh");


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
    viewer_ptr->setBackgroundColor (0, 0, 0);
    viewer_ptr->addCoordinateSystem (1.0);
    viewer_ptr->initCameraParameters ();
    viewer_ptr->setCameraPosition(-5,5,-5, 0,0,0);
    // Add center (0,0,0) sphere
    viewer_ptr->addSphere((pcl::PointXYZ(0.0, 0.0, 0.0)), 0.05, "sphere", 0);
}