/**
 * Everything to do with calculating or estimation volumes
 *  - Calculate box volume
 *  - Calculate mesh volume using 'division into tetrahedrons'
*/

#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>


/*
Calculates the volume of a box specified by its min and max points
*/
float calcBoxVolume(pcl::PointXYZ min_point, pcl::PointXYZ max_point) {
    return (max_point.x - min_point.x)*(max_point.y - min_point.y)*(max_point.z - min_point.z);
}


//TODO: Change cloud_ptr to have generic PointT type instead of specific PointXYZ type
/*
Calculates the volume of the given mesh using a 'division into tetrahedrons' algorithm.
Assumes the surface polygons in 'mesh_ptr' are all triangles.
Assumes the surface normals in 'surface_normals_ptr' are in the same order as the surface polygons in 'mesh"ptr'.
*/
float calcMeshVolumeTetrahedron(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr surface_normals_ptr, pcl::PointXYZ reference_point = pcl::PointXYZ(0.0f, 0.0f, 0.0f)) {
    float volume = 0.0f;

    for (int i = 0; i < mesh_ptr->polygons.size(); i++) { // Assumes each polygon has exactly 3 vertices (= a triangle)
        pcl::Vertices vs = mesh_ptr->polygons[i];
        
        // Retrieve vertices
        pcl::index_t i_v1 = vs.vertices[0];
        pcl::index_t i_v2 = vs.vertices[1];
        pcl::index_t i_v3 = vs.vertices[2];
        pcl::PointXYZ p1 = cloud_ptr->at(i_v1);
        pcl::PointXYZ p2 = cloud_ptr->at(i_v2);
        pcl::PointXYZ p3 = cloud_ptr->at(i_v3);

        // Calculate tetrahedron's volume
        float vol1 = -(p2.x-reference_point.x)*(p3.y-reference_point.y)*(p1.z-reference_point.z)
            + (p3.x-reference_point.x)*(p2.y-reference_point.y)*(p1.z-reference_point.z)
            + (p2.x-reference_point.x)*(p1.y-reference_point.y)*(p3.z-reference_point.z)
            - (p1.x-reference_point.x)*(p2.y-reference_point.y)*(p3.z-reference_point.z)
            - (p3.x-reference_point.x)*(p1.y-reference_point.y)*(p2.z-reference_point.z)
            + (p1.x-reference_point.x)*(p3.y-reference_point.y)*(p2.z-reference_point.z);
        float vol_abs = std::abs(vol1);
        float vol_tetra = vol_abs / 6.0f;

        // Add or subtract current tetrahedron's volume from the total volume
        // based on the sign of the dot product between the surface normal 
        // and the vector from the reference point to v1
        Eigen::Vector3f surface_normal = surface_normals_ptr->at(i).getNormalVector3fMap();
        Eigen::Vector3f vec_ref_1 = p1.getVector3fMap() - reference_point.getVector3fMap();
        float sign = surface_normal.dot(vec_ref_1);
        sign = sign / std::abs(sign); // Turn into just +/-1
        
        float vol_tetra_signed = vol_tetra * sign;

        volume += vol_tetra_signed;
    }

    return volume;
}