// for 'std::cerr'
#include <iostream>

#include <string>
// for std::ifstream
#include <fstream>


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
float readActualVolume(std::string path);