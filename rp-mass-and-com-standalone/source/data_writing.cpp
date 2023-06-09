#include <iostream>
#include <fstream>

/*
Writes the given data string to the given file.
If the file already exists, the contents are overwritten.
*/
int writeStringToFile(std::string &file_path, std::string &data) {
    std::ofstream file;
    file.open(file_path, std::ios::out | std::ios::trunc); // Opens file, deleting file content if it already exists

    if (file.is_open()) {
        std::cerr << "File '" << file_path << "' opened." << std::endl;
    } else {
        std::cerr << "File '" << file_path << "' could not be opened." << std::endl;
    }

    file << data;
    file.close();
    return 0;
}


/*
Appends the given data string to the given file.
If the file does not exist, it is created.
*/
int appendStringToFile(std::string &file_path, std::string &data) {
    std::ofstream file;
    file.open(file_path, std::ios::out | std::ios::app); // Opens file, deleting file content if it already exists

    if (file.is_open()) {
        std::cerr << "File '" << file_path << "' opened." << std::endl;
    } else {
        std::cerr << "File '" << file_path << "' could not be opened." << std::endl;
    }

    file << data;
    file.close();
    return 0;
}