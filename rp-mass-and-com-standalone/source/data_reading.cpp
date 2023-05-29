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
    size_t start_pos;
    // If 'object_file_path' is none of the supported 3d files, return -1
    if ((start_pos = object_file_path.find(".pcd")) == std::string::npos
    && (start_pos = object_file_path.find(".ply")) == std::string::npos
    && (start_pos = object_file_path.find(".obj")) == std::string::npos) {
        return -1;
    }
    

    // Get data file name from 3d object data file name
    // by replacing the .3d_file_extension with '_data.txt'
    
    data_file_path_out = object_file_path; // I think this copies the whole string and not just references the original one (hopefully)
    float vol_actual = -1;
    std::string type;
    data_file_path_out.replace(start_pos, 4, "_data.txt");
    std::cerr << "data_file_path '" << data_file_path_out << "'" << std::endl;

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
    } else {
        // Read volume data
        in >> type >> vol_actual;
        std::cerr << "read data '" << type << " : " << vol_actual << "'" << std::endl;
    }
    in.close();

    return vol_actual;
}