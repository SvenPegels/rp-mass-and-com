#include "data_reading.hpp"

/**
 * Everything for reading custom data from custom files
*/


/*
Reads the actual volume from a _data.txt file.
Return the read volume, or -1 if something went wrong.
Assumes that the file is formatted as such (has to be on the first line):
    ""
    Volume <actual volume>
    ""
    *Any single word instead of 'Volume' will work, but use Volume for clarity and future proofing.

- <arg> 'path'    Full file path to the 3d object data (.pcd / .ply / .obj).
    *Might edit the original string passed as an argument (hopefully not)
*/
float readActualVolume(std::string path) {

    // Verify path point to a supported file, 
    // and find the position of the .file_extension the the path string

    size_t start_pos;
    // If 'path' is none of the supported 3d files, return -1
    if ((start_pos = path.find(".pcd")) == std::string::npos
    && (start_pos = path.find(".ply")) == std::string::npos
    && (start_pos = path.find(".obj")) == std::string::npos) {
        return -1;
    }
    

    // Get data file name from 3d object data file name
    // by replacing the .3d_file_extension with '_data.txt'
    
    std::string data_file_path = path; // I think this copies the whole string and not just references the original one (hopefully)
    float vol_actual = -1;
    std::string type;
    data_file_path.replace(start_pos, 4, "_data.txt");
        std::cerr << "data_file_path '" << data_file_path << "'" << std::endl;
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