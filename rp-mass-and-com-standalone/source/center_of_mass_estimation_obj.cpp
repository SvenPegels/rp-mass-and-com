/**
 * Center of Mass estimation from .obj and .ply files
 * 
 * For code consistency:
 * - Visualizers as ::Ptr
 * - PointClouds/Meshes/etc as ::Ptr
 * - Single points not as ::Ptr. Passed as &reference to functions where necessary
*/

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

/*Own code*/
#include "data_reading.cpp"
#include "bounding_box.cpp"
#include "visualization.cpp"
#include "surface_utils.cpp"
#include "center_of_mass.cpp"


using namespace std::chrono_literals;


void initVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer_ptr);

/*
./volume_estimation_pcd ~/rp-mass-and-com/pcd_files/partial_view_generated_clouds/cylinder_triangulated_lc_cam_1.pcd ~/rp-mass-and-com/obj_files/parti
al_view_base_models/base_quality/data/cylinder_triangulated_lc_data.txt
*/
int main(int argc, char** argv) {
    /*
    Check arguments and retrieve data file path
    */
    std::string data_file_path;
    if (argc >= 3) {
        // .obj and data.txt given
        data_file_path = argv[2];
    } else if (argc >= 2) {
        // .obj or data.txt not given. Likely only .obj
        std::cerr << "No .obj or data file argument given. Assuming .obj is given. Defaulting to _data.txt for data." << std::endl;
        if (dataFilePathFromObjectFilePath(argv[1], data_file_path) != 0) {
            data_file_path = "OBJECT_FILE_NOT_SUPPORTED";
            std::cerr << "Object file type not supported!" << std::endl;
            return -1;
        }
    } else {
        std::cerr << "No .obj file argument given!" << std::endl;
        return -1;
    }


    /*
    Load 3d data
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ()); // TODO: Temp
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);

    std::string threed_file_path = std::string(argv[1]);

    // Load data from .obj, if given file is a .obj file
    if (threed_file_path.find(".obj") != std::string::npos) {
        loadOBJData(threed_file_path, cloud_ptr, cloud_w_normals_ptr, mesh_ptr);
    }
    // Load data from .ply, if given file is a .ply file
    else if (threed_file_path.find(".ply") != std::string::npos) {
        loadPLYData(threed_file_path, cloud_ptr, cloud_w_normals_ptr, mesh_ptr);
    }
    else {
        std::cerr << "Given 3D data file argument is not a .obj or .ply file. Exiting." << std::endl;
        return -1;
    }


    pcl::PointXYZ com_actual = pcl::PointXYZ();
    int res = readActualCoMFromDataFile(data_file_path, com_actual);


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


    // Calculate AABB center point (average of AABB min and max)
    pcl::PointXYZ AABB_center_point = pcl::PointXYZ((AABB_min_point.x+AABB_max_point.x)/2, (AABB_min_point.y+AABB_max_point.y)/2, (AABB_min_point.z+AABB_max_point.z)/2);

    // Calculate OBB center point (average of OBB min and max) * OBB rotation + OBB position
    pcl::PointXYZ OBB_center_point = pcl::PointXYZ((OBB_min_point.x+OBB_max_point.x)/2, (OBB_min_point.y+OBB_max_point.y)/2, (OBB_min_point.z+OBB_max_point.z)/2);
    getOBBPoint(OBB_center_point, OBB_rot, OBB_pos, OBB_center_point);


    /*
    CoM estimation using tetrahedrons
    */

    // Estimate centroid normals
    // Centroid normal is estimated using the cross product of the vectors from p1 to p2, and p1 to p3.
    pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr (new pcl::PointCloud<pcl::Normal>());
    estimateSurfaceNormals(mesh_ptr, cloud_w_normals_ptr, centroid_normals_ptr, true);

    pcl::PointXYZ com_tetra = pcl::PointXYZ();
    res = calcMeshCoMTetrahedron(mesh_ptr, cloud_ptr, centroid_normals_ptr, com_tetra, AABB_center_point);
    std::cerr << "Tetra CoM: '" << com_tetra << "'" << std::endl;




    /*
    Display results
    */
    // Display sizes in the bottom left corner of the screen
    displayText(viewer_ptr, "Actual CoM: " + toString(com_actual, 3));
    displayText(viewer_ptr, "AABB CoM: " + toString(AABB_center_point, 3));
    displayText(viewer_ptr, "OBB CoM: " + toString(OBB_center_point, 3));
    displayText(viewer_ptr, "Tetra CoM: " + toString(com_tetra, 3));
    // displayText(viewer_ptr, "Convex Hull CoM: " + toString(com_chull));


    
    // Run until window is closed
    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

/*
Initializes PCLVisualizer with preferred values
*/
void initVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer_ptr) {
    /*
    X and Z are horizontal
    Y is up
    */
    viewer_ptr->setBackgroundColor (0, 0, 0);
    viewer_ptr->addCoordinateSystem (1.0);
    viewer_ptr->initCameraParameters ();
    viewer_ptr->setCameraPosition(20,20,20, 0,0,0);
    // Add center (0,0,0) sphere
    // viewer_ptr->addSphere((pcl::PointXYZ(0.0, 0.0, 0.0)), 0.05, "sphere", 0);

    // Add and Red, Green and Blue sphere on the X, Y and Z axis respectively
    // viewer_ptr->addSphere(pcl::PointXYZ(20.0, 0.0, 0.0), 0.5, "sphere x");
    // viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "sphere x");
    // viewer_ptr->addSphere(pcl::PointXYZ(0.0, 20.0, 0.0), 0.5, "sphere y");
    // viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sphere y");
    // viewer_ptr->addSphere(pcl::PointXYZ(0.0, 0.0, 20.0), 0.5, "sphere z");
    // viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "sphere z");

}