/**
 * Everything to do with calculation surface features
 *  - Surface centroids
 *  - Surace normals from vertices
 *  - Adjust surface normals based on vertex normals
*/

#include "surface_utils.hpp"


/*
Calculates surface centroids based on polygon vertices.
Assumes all surface polygons are triangles.
*/
void calcSurfaceCentroids(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr surface_centroids_ptr_out) {
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
        surface_centroids_ptr_out->push_back(center);

    }
}


/*
Estimates the surface centroid normals based on the polygon vertices.
Assumes all surface polygons are triangles (have 3 vertices).
*/
void estimateSurfaceNormals(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr, pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr_out, bool correct_normals = false) {
    int n_flipped = 0;
    for (int i = 0; i < mesh_ptr->polygons.size(); i++) { // Assumes each polygon has exactly 3 vertices (= a triangle)
        // For debuggin purposes
        bool debug_print = i < 5 || i > mesh_ptr->polygons.size()-5 || (i >= 500 && i < 505);
        debug_print = false;
        if (debug_print) std::cerr << "i=" << i << std::endl;

        // Retrieve vertices with normals
        pcl::Vertices vs = mesh_ptr->polygons[i];
        pcl::index_t i_v1 = vs.vertices[0];
        pcl::index_t i_v2 = vs.vertices[1];
        pcl::index_t i_v3 = vs.vertices[2];
        pcl::PointNormal p1 = cloud_w_normals_ptr->at(i_v1);
        pcl::PointNormal p2 = cloud_w_normals_ptr->at(i_v2);
        pcl::PointNormal p3 = cloud_w_normals_ptr->at(i_v3);

        if (debug_print) std::cerr << "p1: '" << p1 << "'" << std::endl;
        if (debug_print) std::cerr << "p2: '" << p2 << "'" << std::endl;
        if (debug_print) std::cerr << "p3: '" << p3 << "'" << std::endl;

        if (debug_print) std::cerr << "p1 normal: '(" << p1.normal_x << "," << p1.normal_y << "," << p1.normal_z << ")'" << std::endl;
        if (debug_print) std::cerr << "p1 normEV: '" << p1.getNormalVector3fMap() << "'" << std::endl;
        if (debug_print) std::cerr << "p2 normal: '(" << p2.normal_x << "," << p2.normal_y << "," << p2.normal_z << ")'" << std::endl;
        if (debug_print) std::cerr << "p2 normEV: '" << p2.getNormalVector3fMap() << "'" << std::endl;
        if (debug_print) std::cerr << "p3 normal: '(" << p3.normal_x << "," << p3.normal_y << "," << p3.normal_z << ")'" << std::endl;
        if (debug_print) std::cerr << "p3 normEV: '" << p3.getNormalVector3fMap() << "'" << std::endl;

        // Calculate surface normal
        // by taking the cross procuct of the edge from p2 to p1
        // and the edge from p3 to p1.
        // With meshes triangulated and exported to .obj in blender this should put the normals in the correct direction.
        Eigen::Vector3f vec_p1_p2 = p1.getVector3fMap() - p2.getVector3fMap();
        Eigen::Vector3f vec_p1_p3 = p1.getVector3fMap() - p3.getVector3fMap();;
        Eigen::Vector3f normal = vec_p1_p2.cross(vec_p1_p3);
        normal.normalize();

        //TODO: Uncomment only for testing
        // flipNormal(normal);
        

        // Correct calculated surface normal based on the surface triangle's vertex normals.
        // The calculated normal is flipped if the dot product between the surface normal
        // and the sum of the vertex normals is < 0 (the angle is > 90 degrees). 
        if (correct_normals) {
            Eigen::Vector3f p1_normal = p1.getNormalVector3fMap();
            Eigen::Vector3f p2_normal = p2.getNormalVector3fMap();
            Eigen::Vector3f p3_normal = p3.getNormalVector3fMap();
            // Flip surface normal if the dot product between the calculated centroid normal and the sum of the vertex normals is '< 0'
            Eigen::Vector3f vertex_normal_sum = p1_normal + p2_normal + p3_normal;
            float sign = vertex_normal_sum.dot(normal);
            if (/*normal is towards wrong direction*/ sign < 0.0f) {

                std::cerr << std::endl;
                std::cerr << "i=" << i << " START" << std::endl;
                std::cerr << "p1_normal '" << p1_normal << "'" << std::endl;
                std::cerr << "p2_normal '" << p2_normal << "'" << std::endl;
                std::cerr << "p3_normal '" << p3_normal << "'" << std::endl;
                std::cerr << "sr_normal '" << normal << "'" << std::endl;


                n_flipped++;
                if (debug_print) std::cerr << "FLIPPED" << std::endl;
                normal = Eigen::Vector3f(-1*normal.x(),-1*normal.y(),-1*normal.z());


                std::cerr << "sr_normal '" << normal << "'" << std::endl;
                std::cerr << "i=" << i << " END" << std::endl;
            }

            // Correct direction by pointing surface normal towards the sum vector of the 3 vertices' normals
            // pcl::flipNormalTowardsViewpoint<Eigen::Vector3f>(normal, 
            //     (p1.normal_x + p2.normal_x + p3.normal_x), 
            //     (p1.normal_y + p2.normal_y + p3.normal_y), 
            //     (p1.normal_z + p2.normal_z + p3.normal_z), 
            //     normal.x(), normal.y(), normal.z());
        }
        
        centroid_normals_ptr_out->push_back(pcl::Normal(normal.x(), normal.y(), normal.z()));
        if (debug_print) std::cerr << std::endl;
    }
    //TODO: Remove
    std::cerr << std::endl;
    
    if (correct_normals) std::cerr << "Normals flipped: '" << n_flipped << "'" << std::endl;
}

    
/*
Flips the normals 180 degrees
*/
void flipNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_ptr) {
    for (int i = 0; i < normals_ptr->size(); i++) {
        pcl::Normal normal = normals_ptr->at(i);
        normal.normal_x = -1*normal.normal_x;
        normal.normal_y = -1*normal.normal_y;
        normal.normal_z = -1*normal.normal_z;
    }        
}

/*
Flips the normal 180 degrees
*/
void flipNormal(Eigen::Vector3f &normal) {
    normal = Eigen::Vector3f(-1*normal.x(),-1*normal.y(),-1*normal.z());
}






/*
Greedy triangulation algorithm
*/
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
// void meshFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PolygonMesh::Ptr mesh_res_ptr) {
//     // Estimate normals
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
//     pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ptr (new pcl::search::KdTree<pcl::PointXYZ>);
//     tree_ptr->setInputCloud (cloud_ptr);
//     normal_est.setInputCloud (cloud_ptr);
//     normal_est.setSearchMethod (tree_ptr);
//     normal_est.setKSearch (20);
//     normal_est.compute (*normals_ptr);
//     dp(1);

//     // Concatenate the XYZ and normal fields*
//     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//     pcl::concatenateFields (*cloud_ptr, *normals_ptr, *cloud_with_normals);
//     dp(2);

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
//     gp3.reconstruct (*mesh_res_ptr);
//     // gp3.reconstruct(triangles);
//     dp(5);

//     // Additional vertex information
//     std::vector<int> parts = gp3.getPartIDs();
//     dp(6);
//     std::vector<int> states = gp3.getPointStates();
//     dp(7);

//     std::cerr << "Resulting cloud param: " << mesh_res_ptr->cloud.height << "x" << mesh_res_ptr->cloud.width << std::endl;
//     // std::cerr << "Resulting cloud local: " << triangles.cloud.height << "x" << triangles.cloud.width << std::endl;
//     dp(8);

// }