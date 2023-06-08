/**
 * Center of Mass estimation from .pcd files
 * 
 * For code consistency:
 * - Visualizers as ::Ptr
 * - PointClouds/Meshes/etc as ::Ptr
 * - Single points not as ::Ptr. Passed as &reference to functions where necessary
*/

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <Eigen/Geometry>
#include <pcl/common/centroid.h>

/*Own code*/
#include "data_reading.cpp"
#include "bounding_box.cpp"
#include "visualization.cpp"
#include "surface_utils.cpp"
#include "center_of_mass.cpp"
#include "conversion.cpp"


using namespace std::chrono_literals;


void initVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer_ptr);

/*
./center_of_mass_estimation_pcd ~/rp-mass-and-com/pcd_files/partial_view_generated_clouds/cylinder_triangulated_lc_cam_1.pcd ~/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/data/cylinder_triangulated_lc_data.txt
*/
/**
 * Center of Mass estimation from partial view .pcd files.
*/
int main(int argc, char** argv) {
    /*
    Check arguments and retrieve data file path
    */
    std::string data_file_path;
    if (argc >= 3) {
        // .pcd and data.txt given
        data_file_path = argv[2];
    } else if (argc >= 2) {
        // .pcd or data.txt not given. Likely only .pcd
        std::cerr << "No .pcd or data file argument given. Assuming .obj is given. Defaulting to _data.txt for data." << std::endl;
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
    Load 3d data
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ()); // TODO: Temp
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal> ());

    std::string threed_file_path = std::string(argv[1]);

    // Load data from .pcd, if given file is a .pcd file
    if (threed_file_path.find(".pcd") != std::string::npos) {
        loadPCDData(threed_file_path, cloud_ptr, cloud_w_normals_ptr);
    } else {
        std::cerr << "Given 3D data file argument is not a .pcd file. Exiting." << std::endl;
        return -1;
    }


    pcl::PointXYZ com_actual = pcl::PointXYZ();
    int res = readActualCoMFromDataFile(data_file_path, com_actual);


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

    // Draw bounding boxes
    // drawBoundingBox(viewer_ptr, AABB_min_point, AABB_max_point, 0.0, 0.0, 1.0, "AABB");
    // drawBoundingBox(viewer_ptr, OBB_min_point, OBB_max_point, OBB_pos, OBB_rot, 1.0, 0.0, 0.0, "OBB");

    // Calculate AABB center point (average of AABB min and max)
    pcl::PointXYZ AABB_center_point = pcl::PointXYZ((AABB_min_point.x+AABB_max_point.x)/2, (AABB_min_point.y+AABB_max_point.y)/2, (AABB_min_point.z+AABB_max_point.z)/2);

    // Calculate OBB center point (average of OBB min and max) * OBB rotation + OBB position
    pcl::PointXYZ OBB_center_point = pcl::PointXYZ((OBB_min_point.x+OBB_max_point.x)/2, (OBB_min_point.y+OBB_max_point.y)/2, (OBB_min_point.z+OBB_max_point.z)/2);
    getOBBPoint(OBB_center_point, OBB_rot, OBB_pos, OBB_center_point);


    /*
    Convex Hull
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // calcConvexHull3D(cloud_ptr, convex_hull_cloud_ptr);

    pcl::PolygonMesh::Ptr convex_hull_mesh_ptr (new pcl::PolygonMesh);
    double vol_chull = calcConvexHull3D(cloud_ptr, convex_hull_mesh_ptr);
    
    pcl::fromPCLPointCloud2(convex_hull_mesh_ptr->cloud, *convex_hull_cloud_ptr);

    // Visualize hull points
    viewer_ptr->addPointCloud(convex_hull_cloud_ptr, "convex hull cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "convex hull cloud");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "convex hull cloud");

    // Visualize hull mesh
    viewer_ptr->addPolygonMesh(*convex_hull_mesh_ptr, "convex hull mesh");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.5, "convex hull mesh");
    // viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "convex hull mesh");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, "convex hull mesh");
    // viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "convex hull mesh");

    //TODO: TEMP: Check convex hull face sizes
    std::vector<pcl::Vertices> vertices = convex_hull_mesh_ptr->polygons;
    std::vector<pcl::Vertices> triangulated_vertices = std::vector<pcl::Vertices>();
    for (pcl::Vertices vs : vertices) {
        pcl::Indices inds = vs.vertices;
        std::size_t size = inds.size();
        std::cerr << "Polygon size: '" << size << "'" << std::endl;
    }


    /*
    CoM estimation of the Convex Hull using tetrahedrons
    */
    // Estimate centroid normals
    // Centroid normal is estimated using the cross product of the vectors from p1 to p2, and p1 to p3.
    pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr (new pcl::PointCloud<pcl::Normal>());
    estimateSurfaceNormals(convex_hull_mesh_ptr, cloud_w_normals_ptr, centroid_normals_ptr, true);




    //TODO: Temp: print estimated surface normals
    for (pcl::Normal normal : *centroid_normals_ptr) {
        std::cerr << "Estimated surface normal: '" << normal << "'" << std::endl;
    }
    // Calculate surface centers (centroids)
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_centroids_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    calcSurfaceCentroids(convex_hull_mesh_ptr, cloud_ptr, surface_centroids_ptr);
    
    // Combine centroid xyz and normal
    pcl::PointCloud<pcl::PointNormal>::Ptr surface_centroids_w_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*surface_centroids_ptr, *centroid_normals_ptr, *surface_centroids_w_normals_ptr);

    //TODO: Temp
    viewer_ptr->addPointCloud(surface_centroids_ptr, "hull surface centroids");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "hull surface centroids");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "hull surface centroids");


    /*
    Some normals seem to be pointing towards the outside while others are pointing towards the inside.
    Maybe try correcting them by pointing them away from the center?
    */
    pcl::PointXYZ convex_hull_average = pcl::PointXYZ();
    pcl::CentroidPoint<pcl::PointXYZ> chull_average_calc;
    for (pcl::PointXYZ p : *convex_hull_cloud_ptr) {
        chull_average_calc.add(p);
    }
    chull_average_calc.get(convex_hull_average);
    viewer_ptr->addSphere(convex_hull_average, 0.4, 0.5, 0.5, 0.5, "convex hull average");
    

    pcl::PointCloud<pcl::PointNormal>::Ptr surface_centroids_w_normals_corrected_ptr (new pcl::PointCloud<pcl::PointNormal>());
    correctNormalsAway(surface_centroids_w_normals_ptr, convex_hull_average, surface_centroids_w_normals_corrected_ptr);

    viewer_ptr->addPointCloudNormals<pcl::PointNormal>(surface_centroids_w_normals_ptr, 1, 1.0, "hull centroid normals");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "hull centroid normals");




    pcl::PointXYZ com_hull = pcl::PointXYZ();
    res = calcMeshCoMTetrahedron(convex_hull_mesh_ptr, cloud_ptr, centroid_normals_ptr, com_hull, AABB_center_point);
    std::cerr << "Tetra CoM: '" << com_hull << "'" << std::endl;

    /*
    CoM estimation using the Convex Hull
    */
    // TODO: make dynamic. For now: assumes camera position 1 (-30f, 30f, -30f)
    Eigen::Vector3f cam_pos = Eigen::Vector3f(-30.0, 30.0, -30.0);
    // Rotation of (0,0,0) means looking in the positive z direction
    // NOTE: Quaternion here is (w,x,y,z). In unity it is (x,y,z,w)!!!
    Eigen::Quaternionf cam_rot_quat = Eigen::Quaternionf(0.88112f, 0.27782f, 0.36497f, -0.11508f);
    cam_rot_quat.normalize();

    // Get the direction at which the camera is looking at, represented by a point relative to (0,0,0) at distance 'z'
    Eigen::Vector3f cam_default_dir_rel_pos(0.0, 0.0, 5.0); // looking to positive z-direction by default (0 rotation)
    Eigen::Vector3f cam_dir_rel_pos = cam_rot_quat * cam_default_dir_rel_pos;


    // Visualize camera by a larger red sphere as the position, aiming at a smaller green sphere with a white line as the camera rotation/direction
    pcl::PointXYZ cam_p1(cam_pos.x(), cam_pos.y(), cam_pos.z());
    pcl::PointXYZ cam_p2((cam_pos + cam_dir_rel_pos).x(), (cam_pos + cam_dir_rel_pos).y(), (cam_pos + cam_dir_rel_pos).z());
    viewer_ptr->addLine(cam_p1, cam_p2, 1.0, 1.0, 1.0, "cam line");
    viewer_ptr->addSphere(cam_p1, 1.0, 1.0, 0.0, 0.0, "cam sphere origin");
    viewer_ptr->addSphere(cam_p2, 0.4, 0.0, 1.0, 0.0, "cam sphere direction");

    // Create plane perpendicular to the camera direction, positioned at the furthest away point of the convex hull
    Eigen::Vector3f plane_origin(0.0, 0.0, 0.0); //TODO: Change to furthest hull point
    Eigen::Vector3f plane_normal(-cam_dir_rel_pos.x(), -cam_dir_rel_pos.y(), -cam_dir_rel_pos.z());
    plane_normal.normalize();
    // 1. vector from plane origin to point of interest
    Eigen::Vector3f v = pointXYZtoVector3f(com_hull) - plane_origin;
    // 2. Take the dot product of that vector with the unit plane normal vector 
    float dist = v.dot(plane_normal);
    Eigen::Vector3f projected_point = pointXYZtoVector3f(com_hull) - dist*plane_normal;

    // Visualize projected point
    viewer_ptr->addSphere(vector3fToPointXYZ(projected_point), 0.4, 0.0, 1.0, 0.0, "convex hull com projected");




    /*
    Visualise CoM's
    */
    viewer_ptr->addSphere(com_actual, 1.0, 1.0, 1.0, 1.0, "actual com");
    viewer_ptr->addSphere(com_hull, 0.4, 0.0, 1.0, 0.0, "convex hull com");


    /*
    Display results
    */
    // Display sizes in the bottom left corner of the screen
    displayText(viewer_ptr, "Actual CoM: " + toString(com_actual, 3));
    displayText(viewer_ptr, "AABB CoM: " + toString(AABB_center_point, 3));
    displayText(viewer_ptr, "OBB CoM: " + toString(OBB_center_point, 3));
    displayText(viewer_ptr, "Hull CoM: " + toString(com_hull, 3));
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
    viewer_ptr->addCoordinateSystem (10.0);
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