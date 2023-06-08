/**
 * Everything to do with visuals
*/
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>


// Values for displaying text
static int line_counter = 0;
const int line_height = 12;
const int line_default_offset = 15;


//TODO: Improve documentation
/*
Displays a wireframe bounding box from 'min_point' to 'max_point'.
Useful for dawing AABBs.
*/
void drawBoundingBox(pcl::visualization::PCLVisualizer::Ptr viewer_ptr, pcl::PointXYZ min_point, pcl::PointXYZ max_point, double r, double g, double b, std::string id) {
    viewer_ptr->addCube(min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z, r,g,b, id);
    viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
}


/*
Displays a wireframe bounding box from relative 'min_point' to relative 'max_point', at position 'pos' with rotation 'rot'.
Useful for drawing OBBs.
*/
void drawBoundingBox(pcl::visualization::PCLVisualizer::Ptr viewer_ptr, pcl::PointXYZ min_point, pcl::PointXYZ max_point, pcl::PointXYZ pos, Eigen::Matrix3f rot, double r, double g, double b, std::string id) {
    Eigen::Vector3f position (pos.x, pos.y, pos.z);
    Eigen::Quaternionf rot_quat (rot);
    position = pos.getArray3fMap();

    viewer_ptr->addCube (position, rot_quat, max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z, id);
    viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
    viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, id);
}


/**
 * Displays given text in the bottom left corner, automatically placing it above text previously placed using this function
*/
void displayText(pcl::visualization::PCLVisualizer::Ptr viewer, const std::string &text) {
    viewer->addText(text, 0, line_default_offset + line_counter * line_height);
    line_counter++;
}


/*
Returns a string representation of the given pcl::PointXYZ
*/
std::string toString(pcl::PointXYZ &point, int precision) {
    std::stringstream stream;

    stream << std::fixed << std::setprecision(precision) << point.x;
    std::string x = stream.str();
    stream.str(std::string());

    stream << std::fixed << std::setprecision(precision) << point.y;
    std::string y = stream.str();
    stream.str(std::string());

    stream << std::fixed << std::setprecision(precision) << point.z;
    std::string z = stream.str();
    stream.str(std::string());

    return "(" + x + ", " + y + ", " + z + ")";
}