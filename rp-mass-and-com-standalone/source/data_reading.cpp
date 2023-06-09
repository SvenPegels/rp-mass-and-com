#include "data_reading.hpp"

/**
 * Everything for reading custom data from custom files
*/


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
float readActualVolumeFromObjectFile(std::string object_file_path) {
    // Get data file path
    std::string data_file_path;
    int res = dataFilePathFromObjectFilePath(object_file_path, data_file_path);
    if (res != 0) return -1;

    // Read volume
    float vol_actual = readActualVolumeFromDataFile(data_file_path);
    
    return vol_actual;
}


/*
Retrieves the _data.txt file corresponding to the given .pcd/.plu/.obj object file path.
Returns -1 if an non-supported object file is given, otherwise 0.
*/
int dataFilePathFromObjectFilePath(std::string object_file_path, std::string &data_file_path_out) {
    std::string replacement = "_data.txt";
    int res = replaceObjectFileExtensionWith(object_file_path, replacement, data_file_path_out);
    std::cerr << "data_file_path '" << data_file_path_out << "'" << std::endl;

    return res;
}


/*
Replaces the .pcd, .ply, or .obj file extension with the given string
*/
int replaceObjectFileExtensionWith(std::string &object_file_path, std::string &replacement, std::string &string_out) {
    size_t start_pos;
    // If 'object_file_path' is none of the supported 3d files, return -1
    if ((start_pos = object_file_path.find(".pcd")) == std::string::npos
    && (start_pos = object_file_path.find(".ply")) == std::string::npos
    && (start_pos = object_file_path.find(".obj")) == std::string::npos) {
        return -1;
    }
    

    // Get data file name from 3d object data file name
    // by replacing the .3d_file_extension with '_data.txt'
    
    string_out = object_file_path; // I think this copies the whole string and not just references the original one (hopefully)
    float vol_actual = -1;
    std::string type;
    string_out.replace(start_pos, 4, replacement);

    return 0;
}


/*
Reads the actual volume from a _data.txt file, given the .pcd/.ply/.obj file path.
Returns the volume, or -1 if something went wrong.
Assumes that the file is formatted as such (has to be on the first line):
    ""
    Volume <actual volume>
    ""
    *Any single word instead of 'Volume' will work, but use Volume for clarity and future proofing.
*/
float readActualVolumeFromDataFile(std::string data_file_path) {
    std::string type; // Future proofing to possible expansion of the _data.txt file.
    float vol_actual;
    // Open file
    std::ifstream in(data_file_path);
    if (in.fail()) {
        std::cerr << "Data file '" << data_file_path << "' not found" << std::endl;
        return -1;
    } else {
        // Read Volume data
        in >> type;
        while (type != "Volume") {
            in >> type;
        }
        in >> vol_actual;
        std::cerr << "read data '" << type << " : " << vol_actual << "'" << std::endl;
    }
    in.close();

    return vol_actual;
}


/*
Reads the actual Center of Mass from a _data.txt file, given the .pcd/.ply/.obj file path.
Returns -1 if something went wrong, 0 otherwise.
*/
int readActualCoMFromDataFile(std::string data_file_path, pcl::PointXYZ &com_out) {
    std::string type;
    float x,y,z;

    std::ifstream in(data_file_path);
    if (in.fail()) {
        std::cerr << "Data file '" << data_file_path << "' not found" << std::endl; 
        return -1;
    } else {
        // Read CoM data
        in >> type;
        while (type != "CoM") {
            in >> type;
        }
        in >> x >> y >> z;
        std::cerr << "read data '" << type << " : (" << x << " , " << y << " , " << z << ")'" << std::endl;
        com_out = pcl::PointXYZ(x, y, z);
    }

    return 0;
}


/*
Loads the XYZ point cloud, XYZ point cloud with normals, and mesh from the given .obj file.
returns -1 if something went wrong, 0 otherwise.
*/
int loadOBJData(std::string threed_file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_out, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr_out, pcl::PolygonMesh::Ptr mesh_ptr_out) {
    // Load cloud
    if (pcl::io::loadOBJFile(threed_file_path, *cloud_ptr_out) == 0) {
    std::cerr << ".obj file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud data from given .obj file" << std::endl;
        return -1;
    }
    // Load cloud and normals
    if (pcl::io::loadOBJFile(threed_file_path, *cloud_w_normals_ptr_out) == 0) {
        std::cerr << ".obj file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud and normal data from given .obj file" << std::endl;
        return -1;
    }
    // Load mesh
    if (pcl::io::loadOBJFile(threed_file_path, *mesh_ptr_out) == 0) {
        std::cerr << ".obj file found" << std::endl;
    } else {
        std::cerr << "Could not load mesh data from given .obj file" << std::endl;
        return -1;
    }

    return 0;
}


/*
Loads the XYZ point cloud, XYZ point cloud with normals, and mesh from the given .ply file.
returns -1 if something went wrong, 0 otherwise.
*/
int loadPLYData(std::string threed_file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_out, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr_out, pcl::PolygonMesh::Ptr mesh_ptr_out) {
    // Load cloud
    if (pcl::io::loadPLYFile(threed_file_path, *cloud_ptr_out) == 0) {
    std::cerr << ".ply file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud data from given .ply file" << std::endl;
        return -1;
    }
    // Load cloud and normals
    if (pcl::io::loadPLYFile(threed_file_path, *cloud_w_normals_ptr_out) == 0) {
        std::cerr << ".ply file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud and normal data from given .ply file" << std::endl;
        return -1;
    }
    // Load mesh
    if (pcl::io::loadPLYFile(threed_file_path, *mesh_ptr_out) == 0) {
        std::cerr << ".ply file found" << std::endl;
    } else {
        std::cerr << "Could not load mesh data from given .ply file" << std::endl;
        return -1;
    }

    return 0;
}


/*
Loads the XYZ point cloud and XYZ point cloud with normals.pcd file.
returns -1 if something went wrong, 0 otherwise.
*/
int loadPCDData(std::string threed_file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_out, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals_ptr_out) {
    // Load cloud
    if (pcl::io::loadPCDFile(threed_file_path, *cloud_ptr_out) == 0) {
    std::cerr << ".pcd file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud data from given .pcd file" << std::endl;
        return -1;
    }
    // Load cloud and normals
    if (pcl::io::loadPCDFile(threed_file_path, *cloud_w_normals_ptr_out) == 0) {
        std::cerr << ".pcd file found" << std::endl;
    } else {
        std::cerr << "Could not load point cloud and normal data from given .pcd file" << std::endl;
        return -1;
    }

    return 0;
}