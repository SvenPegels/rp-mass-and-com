/**
 * Everything to do with calculating or estimation Center of Mass
 *  - Calculate mesh CoM using 'division into tetrahedrons'
*/


#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>


/*

*/
int calcMeshCoMTetrahedron(pcl::PolygonMesh::Ptr mesh_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr surface_normals_ptr, pcl::PointXYZ &com_out, pcl::PointXYZ reference_point = pcl::PointXYZ()) {
    // double x_want, y_want, z_want;
    // double x_full, y_full, z_full;
    // double x_extra, y_extra, z_extra;
    // double vol_full, vol_extra;


    double x, y, z;
    double vol_want;

    for (int i = 0; i < mesh_ptr->polygons.size(); i++) {
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

        // Calculate tetrahedron's CoM
        pcl::PointXYZ com_tetra = pcl::PointXYZ(
            (p1.x+p2.x+p3.x+reference_point.x)/4,
            (p1.y+p2.y+p3.y+reference_point.y)/4,
            (p1.z+p2.z+p3.z+reference_point.z)/4);
        
        // Add or subtract current tetrahedron's volume and CoM from the total
        // based on the sign of the dot product between the surface normal
        // and the vector from the reference point to v1
        Eigen::Vector3f surface_normal = surface_normals_ptr->at(i).getNormalVector3fMap();
        Eigen::Vector3f vec_ref_1 = p1.getVector3fMap() - reference_point.getVector3fMap();
        float dot = surface_normal.dot(vec_ref_1);
        float sign = dot / std::abs(dot); // Turn into just +/-1

        float vol_tetra_signed = vol_tetra * sign;
        vol_want += vol_tetra_signed;

        float x_signed = com_tetra.x * sign, y_signed = com_tetra.y * sign, z_signed = com_tetra.z * sign;
        x += x_signed * vol_tetra;
        y += y_signed * vol_tetra;
        z += z_signed * vol_tetra;
    }

    com_out = pcl::PointXYZ(x/vol_want, y/vol_want, z/vol_want);

    ;return 0;
}