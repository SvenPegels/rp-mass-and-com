// for 'std::cerr'
#include <iostream>

#include <string>
// for std::ifstream
#include <fstream>


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
Reads the actual volume from a _data.txt file, given the .pcd/.ply/.obj file path.
Returns the volume, or -1 if something went wrong.
Assumes that the file is formatted as such (has to be on the first line):
    ""
    Volume <actual volume>
    ""
    *Any single word instead of 'Volume' will work, but use Volume for clarity and future proofing.
*/
float readActualVolumeFromDataFile(std::string data_file_path);