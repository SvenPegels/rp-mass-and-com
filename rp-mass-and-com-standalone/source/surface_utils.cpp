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
void estimateSurfaceNormals(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr, pcl::PointCloud<pcl::Normal>::Ptr centroid_normals_ptr_out) {
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

        flipNormal(normal);

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
        centroid_normals_ptr_out->push_back(pcl::Normal(normal.x(), normal.y(), normal.z()));
        if (debug_print) std::cerr << std::endl;
    }
    std::cerr << "Normals flipped: '" << n_flipped << "'" << std::endl;
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