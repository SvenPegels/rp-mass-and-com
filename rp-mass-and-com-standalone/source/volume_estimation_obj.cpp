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

#include <string>
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
./volume_estimation_obj ~/rp-mass-and-com/pcd_files/rotated_torus.obj
*/
/**
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
    // if (pcl::io::loadOBJFile(argv[1], *cloud_w_normals_ptr) == 0) {
    //     std::cerr << ".obj file found" << std::endl;
    // } else {
    //     std::cerr << "Could not load point cloud and normal data from given .obj file" << std::endl;
    //     return -1;
    // }
    // Load mesh data
    if (pcl::io::loadOBJFile(argv[1], *mesh_ptr) == 0) {
        std::cerr << ".obj file found" << std::endl;
    } else {
        std::cerr << "Could not load mesh data from given .obj file" << std::endl;
        return -1;
    }
    pcl::fromPCLPointCloud2(mesh_ptr->cloud, *cloud_ptr);
    pcl::fromPCLPointCloud2(mesh_ptr->cloud, *cloud_w_normals_ptr);
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
    Visualization
    */
    // Basic visualization setup
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    // viewer->addPointCloud(cloud_xyzrgb_ptr);
    // viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud_xyzrgb_w_normals_ptr, 1, 0.05f, "cloud");
    viewer->addPolygonMesh(*mesh_ptr, "polygon mesh");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "polygon mesh");
    
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
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

    //TODO: Uncomment to draw AABB and OBB again
    // // Draw AABB as wireframe
    // viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 0.0, 0.0, "AABB");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
    // // Draw OBB as a cube, at 'position', with rotation 'rot_quat', with vertex lengths, displayed as a wireframe
    // Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    // Eigen::Quaternionf rot_quat (rotational_matrix_OBB);
    // viewer->addCube (position, rot_quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "OBB");


    // Read actual size (assuming file only contains volume data)
    float vol_actual = -1;
    std::string type;
    // Get data file name
    std::string data_file_name = argv[1];
    size_t start_pos = data_file_name.find(".obj"); // Should check using std::string::npos, but would have failed already anyway if not a .obj file
    if (start_pos < data_file_name.length()) { // If 'data_file_name' contains ".obj"
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
    }
    

    // Calculate size
    float vol_AABB = (max_point_AABB.x - min_point_AABB.x)*(max_point_AABB.y - min_point_AABB.y)*(max_point_AABB.z - min_point_AABB.z);
    float vol_OBB = (max_point_OBB.x - min_point_OBB.x)*(max_point_OBB.y - min_point_OBB.y)*(max_point_OBB.z - min_point_OBB.z);


    /*
    Estimate using tetrahedrons
    */
    std::cerr << "Mesh header: '" << mesh_ptr->header << "'" << std::endl;
    std::cerr << "Mesh polygons size: '" << mesh_ptr->polygons.size() << "'" << std::endl;
    pcl::PointXYZ pref (pcl::PointXYZ(0.0, 0.0, 0.0));

    // Estimate surface normals
    // pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals_ptr (new pcl::PointCloud<pcl::PointNormal>());
    // pcl::features::computeApproximateNormals(*cloud_ptr, mesh_ptr->polygons, *surface_normals_ptr);
    // pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals_at_center_ptr (new pcl::PointCloud<pcl::PointNormal>());
    // pcl::features::computeApproximateNormals(*cloud_ptr, mesh_ptr->polygons, *surface_normals_at_center_ptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals_ptr (new pcl::PointCloud<pcl::PointNormal>());

    // TODO: Fix and add to thesis: Produces problem with vertices being oriented towards the viewpoint (as described in bachelors thesis), where viewpoint is assumed to be at (0,0,0)
    pcl::features::computeApproximateNormals(*cloud_ptr, mesh_ptr->polygons, *surface_normals_ptr);
    // surface normals are added to the .getNormalVector3fMap()


    // TODO: 1152 surfaces (triangles), but only 576 surface normals calculated. 576 is the original amount of surfaces (still squares). 1152 is after triangulaization (square -> triangles)



    // Calculate surface centers
    int i = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_centers_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < mesh_ptr->polygons.size(); i++) { // Assumes each polygon has exactly 3 vertices (= a triangle)
        // Retrieve vertices
        pcl::Vertices vs = mesh_ptr->polygons[i];
        pcl::index_t i_v1 = vs.vertices[0];
        pcl::index_t i_v2 = vs.vertices[1];
        pcl::index_t i_v3 = vs.vertices[2];
        pcl::PointXYZ p1 = cloud_ptr->at(i_v1);
        pcl::PointXYZ p2 = cloud_ptr->at(i_v2);
        pcl::PointXYZ p3 = cloud_ptr->at(i_v3);
        // pcl::PointXYZ center = ;
        float center_x = (p1.x+p2.x+p3.x)/3;
        float center_y = (p1.y+p2.y+p3.y)/3;
        float center_z = (p1.z+p2.z+p3.z)/3;
        pcl::PointXYZ center (pcl::PointXYZ(center_x, center_y, center_z));
        surface_centers_ptr->push_back(center);
        // if (i < 5) std::cerr << "p1: '" << p1 << "'" << std::endl;
        // if (i < 5) std::cerr << "p2: '" << p2 << "'" << std::endl;
        // if (i < 5) std::cerr << "p3: '" << p3 << "'" << std::endl;
        // if (i < 5) std::cerr << "center: '" << center << "'" << std::endl;

        // Add surface center coord to normals to make normals point in correct direction
        // surface_normals_ptr->at(i).normal_x += center_x;
        // surface_normals_ptr->at(i).normal_y += center_y;
        // surface_normals_ptr->at(i).normal_z += center_z;
        std::cerr << "" << i << std::endl;
        i++;

        // Calculate sign
        // = Inner product between vector from p1 to pref, and the face normal

    }

    // Display normals at the surface center
    // std::cerr << "before adding surface normals" << std::endl;
    std::cerr << "n_mesh_polygons : '" << mesh_ptr->polygons.size() << "'" << std::endl;
    std::cerr << "n_surface_centers : '" << surface_centers_ptr->size() << "'" << std::endl;
    std::cerr << "n_surface_normals : '" << surface_normals_ptr->size() << "'" << std::endl;
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(surface_centers_ptr, surface_normals_ptr, 1, 0.05f, "surface normals"); //TODO: Add normal points
    // // viewer->addPointCloudNormals<pcl::PointNormal>(surface_normals_ptr, 5, 0.05f, "surface normals 2");

    // std::cerr << "after adding surface normals" << std::endl;
    

    // Calculate volume
    pcl::index_t offset = 0;
    float vol_tetra = 0.0;
    i = 0;
    for (pcl::Vertices vs : mesh_ptr->polygons) { // Assumes each polygon has exactly 3 vertices (= a triangle)
        // Retrieve vertices
        pcl::index_t i_v1 = vs.vertices[0];
        pcl::index_t i_v2 = vs.vertices[1];
        pcl::index_t i_v3 = vs.vertices[2];
        pcl::PointXYZ p1 = cloud_ptr->at(i_v1);
        pcl::PointXYZ p2 = cloud_ptr->at(i_v2);
        pcl::PointXYZ p3 = cloud_ptr->at(i_v3);
        // mesh_ptr->cloud.at(i_v1, offset);
        // mesh_ptr->cloud.data[i_v1];
        // cloud_ptr->at(i_v1).x;
        // Calculate tetrahedron's volume
        float vol = -(p2.x-pref.x)*(p3.y-pref.y)*(p1.z-pref.z)
            + (p3.x-pref.x)*(p2.y-pref.y)*(p1.z-pref.z)
            + (p2.x-pref.x)*(p1.y-pref.y)*(p3.z-pref.z)
            - (p1.x-pref.x)*(p2.y-pref.y)*(p3.z-pref.z)
            - (p3.x-pref.x)*(p1.y-pref.y)*(p2.z-pref.z)
            + (p1.x-pref.x)*(p3.y-pref.y)*(p2.z-pref.z);
        float vol2 = std::abs(vol);
        float vol3 = vol2 / 6.0f;
        // if (i < 10) std::cerr << "vol: '" << vol3 << "'" << std::endl;
        i++;

        // Calculate sign
        // = Inner product between vector from p1 to pref, and the face normal




        vol_tetra += vol3;
    }
    std::cerr << "Total tetra vol: '" << vol_tetra << "'" << std::endl;

    


    // Display size
    displayText(viewer, "Actual volume: " + std::to_string(vol_actual));
    displayText(viewer, "AABB volume: " + std::to_string(vol_AABB));
    displayText(viewer, "OBB volume: " + std::to_string(vol_OBB));
    displayText(viewer, "Tetra volume: " + std::to_string(vol_tetra));




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