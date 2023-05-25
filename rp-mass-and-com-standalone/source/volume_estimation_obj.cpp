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
    Visualization
    */
    // Basic visualization setup
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
    drawOBB(viewer_ptr, OBB_min_point, OBB_max_point, OBB_pos, OBB_rot, 1.0, 0.0, 0.0, "OBB");

    // Read actual size (assuming file only contains volume data)
    float vol_actual = readActualVolume(argv[1]);
    
    // Calculate size
    float vol_AABB = (AABB_max_point.x - AABB_min_point.x)*(AABB_max_point.y - AABB_min_point.y)*(AABB_max_point.z - AABB_min_point.z);
    float vol_OBB = (OBB_max_point.x - OBB_min_point.x)*(OBB_max_point.y - OBB_min_point.y)*(OBB_max_point.z - OBB_min_point.z);
    

    /*
    Estimate using tetrahedrons
    */
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
    // I think this calculates the surface normal by

    (*surface_normals_ptr)[0].getNormalVector3fMap();


    // Calculate surface centers (centroids)
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_centroids_ptr (new pcl::PointCloud<pcl::PointXYZ>);
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
        surface_centroids_ptr->push_back(center);
        // if (i < 5) std::cerr << "p1: '" << p1 << "'" << std::endl;
        // if (i < 5) std::cerr << "p2: '" << p2 << "'" << std::endl;
        // if (i < 5) std::cerr << "p3: '" << p3 << "'" << std::endl;
        // if (i < 5) std::cerr << "center: '" << center << "'" << std::endl;

        // Add surface center coord to normals to make normals point in correct direction
        // surface_normals_ptr->at(i).normal_x += center_x;
        // surface_normals_ptr->at(i).normal_y += center_y;
        // surface_normals_ptr->at(i).normal_z += center_z;
        // std::cerr << "" << i << std::endl;

        // Calculate sign
        // = Inner product between vector from p1 to pref, and the face normal

    }

    std::cerr << std::endl;

    // Estimate centroid normals
    // Centroid normal is estimated using the cross product of the vectors from p1 to p2, and p1 to p3.
    int n_flipped = 0;
    pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr (new pcl::PointCloud<pcl::Normal>());
    for (int i = 0; i < mesh_ptr->polygons.size(); i++) { // Assumes each polygon has exactly 3 vertices (= a triangle)
        bool debug_print = i < 5 || i > mesh_ptr->polygons.size()-5 || (i >= 500 && i < 505);
        debug_print = false;
        if (debug_print) std::cerr << "i=" << i << std::endl;
        // Retrieve vertices
        pcl::Vertices vs = mesh_ptr->polygons[i];
        pcl::index_t i_v1 = vs.vertices[0];
        pcl::index_t i_v2 = vs.vertices[1];
        pcl::index_t i_v3 = vs.vertices[2];
        pcl::PointNormal p1 = cloud_w_normals_ptr->at(i_v1);
        pcl::PointNormal p2 = cloud_w_normals_ptr->at(i_v2);
        pcl::PointNormal p3 = cloud_w_normals_ptr->at(i_v3);
        pcl::PointXYZ centroid = surface_centroids_ptr->at(i);

        if (debug_print) std::cerr << "p1: '" << p1 << "'" << std::endl;
        if (debug_print) std::cerr << "p2: '" << p2 << "'" << std::endl;
        if (debug_print) std::cerr << "p3: '" << p3 << "'" << std::endl;

        if (debug_print) std::cerr << "p1 normal: '(" << p1.normal_x << "," << p1.normal_y << "," << p1.normal_z << ")'" << std::endl;
        if (debug_print) std::cerr << "p1 normEV: '" << p1.getNormalVector3fMap() << "'" << std::endl;
        if (debug_print) std::cerr << "p2 normal: '(" << p2.normal_x << "," << p2.normal_y << "," << p2.normal_z << ")'" << std::endl;
        if (debug_print) std::cerr << "p2 normEV: '" << p2.getNormalVector3fMap() << "'" << std::endl;
        if (debug_print) std::cerr << "p3 normal: '(" << p3.normal_x << "," << p3.normal_y << "," << p3.normal_z << ")'" << std::endl;
        if (debug_print) std::cerr << "p3 normEV: '" << p3.getNormalVector3fMap() << "'" << std::endl;

        if (debug_print) std::cerr << "center: '" << centroid << "'" << std::endl;

        // Calculate surface normal
        Eigen::Vector3f vec_p1_p2 = p1.getVector3fMap() - p2.getVector3fMap();
        Eigen::Vector3f vec_p1_p3 = p1.getVector3fMap() - p3.getVector3fMap();;
        Eigen::Vector3f normal = vec_p1_p2.cross(vec_p1_p3);
        normal.normalize();


        //TODO: REMOVE (TEMPORARY). Flips normals to check if correction below works correctly
        normal = Eigen::Vector3f(-1*normal.x(),-1*normal.y(),-1*normal.z());
        //TODO: END REMOVE


        if (debug_print) std::cerr << "cent. normal EV (non-corrected): '" << normal << "'" << std::endl;
        
        Eigen::Vector3f p1_normal = p1.getNormalVector3fMap();
        Eigen::Vector3f p2_normal = p2.getNormalVector3fMap();
        Eigen::Vector3f p3_normal = p3.getNormalVector3fMap();
        // Flip centroid normal if the dot product between the calculated centroid normal and the sum of the vertex normals is '< 0'
        Eigen::Vector3f vertex_normal_sum = p1_normal + p2_normal + p3_normal;
        float sign = vertex_normal_sum.dot(normal);
        if (/*normal is wrong*/ sign < 0.0f) {
            n_flipped++;
            if (debug_print) std::cerr << "FLIPPED" << std::endl;
            normal = Eigen::Vector3f(-1*normal.x(),-1*normal.y(),-1*normal.z());
        }

        if (debug_print) std::cerr << "vert. normal sum EV: '" << vertex_normal_sum << "'" << std::endl;
        if (debug_print) std::cerr << "cent. normal EV (corrected): '" << normal << "'" << std::endl;

        // Correct direction by pointing surface normal towards the sum vector of the 3 vertices' normals
        // pcl::flipNormalTowardsViewpoint<Eigen::Vector3f>(normal, 
        //     (p1.normal_x + p2.normal_x + p3.normal_x), 
        //     (p1.normal_y + p2.normal_y + p3.normal_y), 
        //     (p1.normal_z + p2.normal_z + p3.normal_z), 
        //     normal.x(), normal.y(), normal.z());
        centroid_normals_ptr->push_back(pcl::Normal(normal.x(), normal.y(), normal.z()));
        if (debug_print) std::cerr << std::endl;
    }
    std::cerr << "n_flipped: '" << n_flipped << "' out of '" << mesh_ptr->polygons.size() << "' polygons." << std::endl;
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
    float vol_tetra = 0.0;
    int i = 0;
    for (int i = 0; i < mesh_ptr->polygons.size(); i++) { // Assumes each polygon has exactly 3 vertices (= a triangle)
        pcl::Vertices vs = mesh_ptr->polygons[i];
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
        Eigen::Vector3f surface_normal = centroid_normals_ptr->at(i).getNormalVector3fMap();
        Eigen::Vector3f vec_0_1 = p1.getVector3fMap() - pref.getVector3fMap();
        float sign = surface_normal.dot(vec_0_1);
        sign = sign / std::abs(sign);
        // Calculate sign
        // = Inner product between vector from p1 to pref, and the face normal
        //TODO: Get Eigen::vector3f's of both, then .dot(), then use sign of that
        float vol4 = vol3 * sign;


        // if (i < 5) std::cerr << "vol3: '" << vol3 << "'" << std::endl;
        // if (i < 5) std::cerr << "sign: '" << sign << "'" << std::endl;
        // if (i < 5) std::cerr << "vol4: '" << vol4 << "'" << std::endl;


        vol_tetra += vol4;
    }
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