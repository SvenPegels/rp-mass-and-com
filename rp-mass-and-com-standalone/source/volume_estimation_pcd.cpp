#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_types.h>

#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

#include <string>
// #include <sstring>
#include <fstream>

#define OUTPUT_DIR ((std::string)"output/")

using namespace std::chrono_literals;

// void drawWireframeBox(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointXYZ min, pcl::PointXYZ max, pcl::PointXYZ position);
// void drawWireframeBox(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointXYZ min, pcl::PointXYZ max, pcl::PointXYZ position, Eigen::Matrix3f rotation);
// void calcAABBfromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
// void estimateOBBromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
// void viewerOneOff (pcl::visualization::PCLVisualizer& viewer);
// void viewerPsycho (pcl::visualization::PCLVisualizer& viewer);
void meshFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PolygonMesh::Ptr mesh_res_ptr);
pcl::PolygonMesh meshFromPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
pcl::PolygonMesh::Ptr meshFromPointCloud3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
void displayText(pcl::visualization::PCLVisualizer::Ptr viewer, const std::string &text);
void dp(int n);

static int line_counter = 0;
const int line_height = 12;
const int line_default_offset = 15;


/*
./volume_estimation ~/rp-mass-and-com/pcd_files/rotated_scaled_cube.pcd
*/
int main (int argc, char** argv) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    // Load cloud data
    if (pcl::io::loadPCDFile(argv[1], *cloud_ptr) == -1) {
        std::cerr << "No .pcd file given" << std::endl;
        return -1;
    }


    /*
    Visualization
    */
    // Convert cloud xyz to xyzrgb
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*cloud_ptr, *cloud_xyzrgb_ptr);
    for (int i = 0; i < (*cloud_xyzrgb_ptr).points.size(); i++) {
        (*cloud_xyzrgb_ptr).points[i].r = 255;
        (*cloud_xyzrgb_ptr).points[i].g = 0;
        (*cloud_xyzrgb_ptr).points[i].b = 0;
    }

    // Basic visualization setup
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
        // viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
    viewer->addPointCloud(cloud_xyzrgb_ptr);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(-5,5,-5, 0,0,0);
    // Add center (0,0,0) sphere
    viewer->addSphere((pcl::PointXYZ(0.0, 0.0, 0.0)), 0.05, "sphere", 0);
    

    /*
    AABB and OBB
    */
    // Data calc setup
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_ptr);
    feature_extractor.compute();

    // Extract AABB data
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);

    // Extract OBB data
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    // Draw AABB as wireframe
    viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 1.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
    // Draw OBB as a cube, at 'position', with rotation 'rot_quat', with vertex lengths, displayed as a wireframe
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf rot_quat (rotational_matrix_OBB);
    viewer->addCube (position, rot_quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");


    // Read actual size (assuming file only contains volume data)
    float vol_actual = -1;
    std::string type;
    // Get data file name
    std::string data_file_name = argv[1];
    size_t start_pos = data_file_name.find(".pcd"); // Should check using std::string::npos, but would have failed already anyway if not a .pcd file
    data_file_name.replace(start_pos, 4, "_data.txt");
    std::cerr << "data_file_name '" << data_file_name << "'" << std::endl;
    // Open file
    std::ifstream in(data_file_name);
    if (in.fail()) {
        std::cerr << "Data file '" << data_file_name << "' not found" << std::endl; 
    } else {
        in >> type >> vol_actual;
        std::cerr << "read data '" << type << " : " << vol_actual << "'" << std::endl;
    }
    in.close();

    // Calculate size
    float vol_AABB = (max_point_AABB.x - min_point_AABB.x)*(max_point_AABB.y - min_point_AABB.y)*(max_point_AABB.z - min_point_AABB.z);
    float vol_OBB = (max_point_OBB.x - min_point_OBB.x)*(max_point_OBB.y - min_point_OBB.y)*(max_point_OBB.z - min_point_OBB.z);

    // Display size
    displayText(viewer, "Actual volume: " + std::to_string(vol_actual));
    displayText(viewer, "AABB volume: " + std::to_string(vol_AABB));
    displayText(viewer, "OBB volume: " + std::to_string(vol_OBB));




    {
        /*
        Greedy triangulation
        */
        pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);
        pcl::PolygonMesh mesh;
        dp(0);
        meshFromPointCloud(cloud_ptr, mesh_ptr);
        std::cerr << "Resulting cloud param: " << mesh_ptr->cloud.height << "x" << mesh_ptr->cloud.width << std::endl;
        mesh = meshFromPointCloud2(cloud_ptr);
        std::cerr << "Resulting cloud param: " << mesh.cloud.height << "x" << mesh.cloud.width << std::endl;
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

        // viewer->addPointCloudNormals(normals_ptr);

        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*cloud_ptr, *normals_ptr, *cloud_with_normals);
        dp(2);

        // viewer->addPointCloudNormals(cloud_with_normals);
        viewer->addPointCloudNormals<pcl::PointNormal>(cloud_with_normals, 1, 0.05f, "cloud with normals");

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
        // gp3.reconstruct (*mesh_ptr);
        gp3.reconstruct(mesh);
        dp(5);

        // Additional vertex information
        std::vector<int> parts = gp3.getPartIDs();
        dp(6);
        std::vector<int> states = gp3.getPointStates();
        dp(7);

        std::cerr << "Resulting cloud param: " << mesh_ptr->cloud.height << "x" << mesh_ptr->cloud.width << std::endl;
        // std::cerr << "Resulting cloud local: " << triangles.cloud.height << "x" << triangles.cloud.width << std::endl;
        dp(8);

        // Display mesh
        viewer->addPolygonMesh(*mesh_ptr);
        viewer->addPolygonMesh(mesh);
    
    }
    dp(00);
    

    // Run until window is closed
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
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

pcl::PolygonMesh meshFromPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
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
    pcl::PolygonMesh triangles;

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
    // gp3.reconstruct (*mesh_res_ptr);
    gp3.reconstruct(triangles);
    dp(5);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    dp(6);
    std::vector<int> states = gp3.getPointStates();
    dp(7);

    // std::cerr << "Resulting cloud param: " << mesh_res_ptr->cloud.height << "x" << mesh_res_ptr->cloud.width << std::endl;
    std::cerr << "Resulting cloud local: " << triangles.cloud.height << "x" << triangles.cloud.width << std::endl;
    dp(8);
    return triangles;
}


pcl::PolygonMesh::Ptr meshFromPointCloud3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
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
    pcl::PolygonMesh::Ptr triangles;

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
    // gp3.reconstruct (*mesh_res_ptr);
    gp3.reconstruct(*triangles);
    dp(5);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    dp(6);
    std::vector<int> states = gp3.getPointStates();
    dp(7);

    // std::cerr << "Resulting cloud param: " << mesh_res_ptr->cloud.height << "x" << mesh_res_ptr->cloud.width << std::endl;
    std::cerr << "Resulting cloud local: " << triangles->cloud.height << "x" << triangles->cloud.width << std::endl;
    dp(8);
    return triangles;
}

/**
 * Displays given text in the bottom left corner, automatically placing it above previously displayed text
*/
void displayText(pcl::visualization::PCLVisualizer::Ptr viewer, const std::string &text) {
    viewer->addText(text, 0, line_default_offset + line_counter * line_height);
    std::cerr << line_counter << std::endl;
    line_counter++;
}


void dp(int n) {
    std::cerr << "Here " << n << std::endl;
}