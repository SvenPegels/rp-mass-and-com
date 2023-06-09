// for 'std::cerr'
#include <iostream>

#include <string>
// for std::ifstream
#include <fstream>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>


/*
Reads the actual volume from a _data.txt file, given the .pcd/.ply/.obj file path.
Returns the volume, or -1 if something went wrong.
Assumes that the file is formatted as such (has to be on the first line):
    ""
    Volume <actual volume>
    ""
    *Any single word instead of 'Volume' will work, but use Volume for clarity and future proofing.

- <arg> 'object_file_path'    Full file path to the 3d object data (.pcd / .ply / .obj).
    *Might edit the original string passed as an argument (hopefully not)
*/
float readActualVolumeFromObjectFile(std::string object_file_path);


/*
Retrieves the _data.txt file corresponding to the given .pcd/.plu/.obj object file path.
Returns -1 if an non-supported object file is given, otherwise 0.
*/
int dataFilePathFromObjectFilePath(std::string object_file_path, std::string &data_file_path_out);


/*
Replaces the .pcd, .ply, or .obj file extension with the given string
*/
int replaceObjectFileExtensionWith(std::string &object_file_path, std::string &replacement, std::string &string_out);


/*
Reads the actual volume from a _data.txt file, given the .pcd/.ply/.obj file path.
Returns the volume, or -1 if something went wrong.
Assumes that the file is formatted as such (has to be on the first line):
    ""
    Volume <actual volume>
    ""
    *Any single word instead of 'Volume' will work, but use Volume for clarity and future proofing.
*/
float readActualVolumeFromDataFile(std::string data_file_path);


/*
Reads the actual Center of Mass from a _data.txt file, given the .pcd/.ply/.obj file path.
Returns -1 if something went wrong, 0 otherwise.
*/
int readActualCoMFromDataFile(std::string data_file_path, pcl::PointXYZ &com_out);


/*
Loads the XYZ point cloud, XYZ point cloud with normals, and mesh from the given .obj file.
returns -1 if something went wrong, 0 otherwise.
*/
int loadOBJData(std::string threed_file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_out, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr_out, pcl::PolygonMesh::Ptr mesh_ptr_out);


/*
Loads the XYZ point cloud, XYZ point cloud with normals, and mesh from the given .ply file.
returns -1 if something went wrong, 0 otherwise.
*/
int loadPLYData(std::string threed_file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_out, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr_out, pcl::PolygonMesh::Ptr mesh_ptr_out);


/*
Loads the XYZ point cloud and XYZ point cloud with normals.pcd file.
returns -1 if something went wrong, 0 otherwise.
*/
int loadPCDData(std::string threed_file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_out, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr_out);