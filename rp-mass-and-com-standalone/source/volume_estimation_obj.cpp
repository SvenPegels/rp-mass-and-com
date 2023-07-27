/**
 * Volume estimation from .obj and .ply files
 * 
 * For code consistency:
 * - Visualizers as ::Ptr
 * - PointClouds/Meshes/etc as ::Ptr
 * - Single points not as ::Ptr. Passed as &reference to functions where necessary
*/

#include <pcl/point_types.h>

#include <iostream>
#include <thread>
#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <sstream>
#include <boost/filesystem.hpp>

// Surface normal estimation

#include <pcl/common/transforms.h>

/*Own code*/
#include "data_reading.cpp"
#include "visualization.cpp"
#include "bounding_box.cpp"
#include "volume_estimation.cpp"
#include "surface_utils.cpp"
#include "data_writing.cpp"

#define OUTPUT_DIR ((std::string)"output/")

using namespace std::chrono_literals;

void dp(int n);
void initVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer_ptr);


/*
./volume_estimation_obj true /home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/high_quality/ply/cone_triangulated_hc.ply /home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/high_quality/data/cone_triangulated_hc_data.txt /home/svenp/rp-mass-and-com/test_results/volume_full_mesh/

./volume_estimation_obj false ~/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/ply/torus_triangulated_lc.ply ~/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/data/torus_triangulated_lc_data.txt
*/
/**
 * Usage: ./volume_estimation_obj <write results> <.obj file> (<data file>) (<results output folder>)
 * Supports .obj and .ply files.
 * If <print results> is set to "true", calculated results will be written to a file in <results output folder> instead, and the will be no visualization.
 * If <print results> is set to "true", both <data file> and <results output folder> must be given.
 * If no <data file> is passed, the file path will be assumed from the <.obj file>.
*/
int main (int argc, char** argv) {
    /*
    Check arguments and retrieve data file path
    */
    if (argc < 3) {
        std::cerr << "Not enough arguments given!" << std::endl;
        return -1;
    }

    std::string arg_write_results = argv[1];
    std::string threed_file_path = std::string(argv[2]);
    std::string data_file_path;
    if (argc >= 4) {
        // .obj and data.txt given
        data_file_path = argv[3];
    } else if (argc >= 3) {
        // .obj or data.txt not given. Likely only .obj
        std::cerr << "No .obj or data file argument given. Assuming .obj is given. Defaulting to _data.txt for data." << std::endl;
        if (dataFilePathFromObjectFilePath(threed_file_path, data_file_path) != 0) {
            data_file_path = "OBJECT_FILE_NOT_SUPPORTED";
            std::cerr << "Object file type not supported!" << std::endl;
            return -1;
        }
    } else {
        std::cerr << "No .obj file argument given!" << std::endl;
        return -1;
    }

    // Check whether to write results to file or visualise them
    bool write_results = false;
    std::string results_output_folder;
    if (arg_write_results == "true") {
        if (argc < 5) {
            std::cerr << "<write results> was 'true' but not enough arguments given!" << std::endl;
        return -1;
        }
        write_results = true;
        results_output_folder = argv[4];
    }


    /*
    Load 3d data
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ()); // TODO: Temp
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);

    // Load data from .obj, if given file is a .obj file
    if (threed_file_path.find(".obj") != std::string::npos) {
        loadOBJData(threed_file_path, cloud_ptr, cloud_w_normals_ptr, mesh_ptr);
    }

    // Load data from .ply, if given file is a .ply file
    if (threed_file_path.find(".ply") != std::string::npos) {
        loadPLYData(threed_file_path, cloud_ptr, cloud_w_normals_ptr, mesh_ptr);
    }

    // pcl::fromPCLPointCloud2(mesh_ptr->cloud, *cloud_ptr);
    // pcl::fromPCLPointCloud2(mesh_ptr->cloud, *cloud_w_normals_ptr);

    // Read actual size (assuming file only contains volume data)
    float vol_actual = readActualVolumeFromDataFile(data_file_path);


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

    std::cerr << "AABB " << (AABB_max_point.x - AABB_min_point.x) << " " << (AABB_max_point.y - AABB_min_point.y) << " " << (AABB_max_point.z - AABB_min_point.z) << std::endl;
    std::cerr << "OBB " << (OBB_max_point.x - OBB_min_point.x) << " " << (OBB_max_point.y - OBB_min_point.y) << " " << (OBB_max_point.z - OBB_min_point.z) << std::endl;

    // Calculate AABB center point (average of AABB min and max)
    pcl::PointXYZ AABB_center_point = pcl::PointXYZ((AABB_min_point.x+AABB_max_point.x)/2, (AABB_min_point.y+AABB_max_point.y)/2, (AABB_min_point.z+AABB_max_point.z)/2);
    

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
    estimateSurfaceNormals(mesh_ptr, cloud_w_normals_ptr, centroid_normals_ptr, true);
    
    // Combine centroid xyz and normal
    pcl::PointCloud<pcl::PointNormal>::Ptr surface_centroids_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*surface_centroids_ptr, *centroid_normals_ptr, *surface_centroids_w_normals_ptr);

    // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(surface_centroids_ptr, centroid_normals_ptr, 1, 0.05f, "centroid normals");
    viewer_ptr->addPointCloudNormals<pcl::PointNormal>(surface_centroids_w_normals_ptr, 1, 0.05f, "centroid normals");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.0, 0.0, "centroid normals");

    // Calculate volume
    float vol_tetra = calcMeshVolumeTetrahedron(mesh_ptr, cloud_ptr, centroid_normals_ptr, AABB_center_point);
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

    // // Create translated hull mesh
    // pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_mesh_points_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(convex_hull_mesh_ptr->cloud, *convex_hull_mesh_points_ptr);
    
    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // transform.translation() << 2.5, 0.0, 0.0;

    // // Translate and visualise translated hull mesh
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // pcl::transformPointCloud (*convex_hull_mesh_points_ptr, *transformed_cloud, transform);
    // viewer_ptr->addPolygonMesh<pcl::PointXYZ>(transformed_cloud, convex_hull_mesh_ptr->polygons, "convex hull mesh translated");

    // Visualize hull mesh
    viewer_ptr->addPolygonMesh(*convex_hull_mesh_ptr, "convex hull mesh");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.5, "convex hull mesh");


    /*
    Display results
    */
    // Display sizes in the bottom left corner of the screen
    displayText(viewer_ptr, "Actual volume: " + std::to_string(vol_actual));
    displayText(viewer_ptr, "AABB volume: " + std::to_string(vol_AABB));
    displayText(viewer_ptr, "OBB volume: " + std::to_string(vol_OBB));
    displayText(viewer_ptr, "Tetra volume: " + std::to_string(vol_tetra));
    displayText(viewer_ptr, "Convex Hull volume: " + std::to_string(vol_chull));
    


    /*
    Visualise or write results
    */
    if (write_results) {
        //TODO: Clean this mess up
        /*
        Output the results to a .csv file
        */
        boost::filesystem::path results_dir_path(results_output_folder);
        // If 'results_output_folder' ends with a '/', the path will point to '.' instead of the actual folder. Taking the parent path solves that.
        if (results_dir_path.filename_is_dot()) {
            results_dir_path = results_dir_path.parent_path();
        }

        std::cerr << "Output folder: '" << results_dir_path.filename() << "'" << std::endl;
        std::cerr << "Output folder: '" << results_dir_path.filename().string() << "'" << std::endl;
        std::cerr << "Output folder path: '" << results_dir_path.string() << "'" << std::endl;

        // Get the input file name from the input file path
        boost::filesystem::path threed_file_path_temp(threed_file_path);
        if (threed_file_path_temp.filename_is_dot()) {
            threed_file_path_temp = threed_file_path_temp.parent_path();
        }
        std::string threed_file_name = threed_file_path_temp.filename().string();
        std::cerr << "treed_file_name: '" << threed_file_name << "'" << std::endl;

        // Get the results file name from the treed file name
        std::string results_file_name;
        std::string results_file_ending = "_results.csv";
        replaceObjectFileExtensionWith(threed_file_name, results_file_ending, results_file_name);

        // Uncomment when writing all results to a single file
        results_file_name = "results.csv";
        
        // Combine the results dir with the results file
        std::string results_file_path = results_dir_path.append(results_file_name).string();
        std::cerr << "Output file: '" << results_file_path << "'" << std::endl;

        // Write the results to a .csv file
        std::stringstream data;
        // data << "file_name" << "," << "actual_volume" << "," << "aabb_volume" << "," << "obb_volume" << "," << "tetra_volume" << "," << "chull_volume" << "\n";
        data << threed_file_path << "," << vol_actual << "," << vol_AABB << "," << vol_OBB << "," << vol_tetra << "," << vol_chull << "\n";
        std::string data_string = data.str();
        // writeStringToFile(results_file_path, data_string);
        appendStringToFile(results_file_path, data_string);
    } else {
        // Run until window is closed
        while (!viewer_ptr->wasStopped()) {
            viewer_ptr->spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }
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