
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>


// Values for displaying text
static int line_counter = 0;
const int line_height = 12;
const int line_default_offset = 15;


//TODO: Improve documentation
/*
Displays a wireframe AABB from 'min_point' to 'max_point'
*/
void drawAABB(pcl::visualization::PCLVisualizer viewer, pcl::PointXYZ min_point, pcl::PointXYZ max_point, double r, double g, double b, std::string id) {
    bool res = viewer.addCube(min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z, r,g,b, id);
    std::cerr << "AABB drawn? '" << res << "'" << std::endl;
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
}

void drawAABB(pcl::visualization::PCLVisualizer::Ptr viewer_ptr, pcl::PointXYZ min_point, pcl::PointXYZ max_point, double r, double g, double b, std::string id) {
    bool res = viewer_ptr->addCube(min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z, r,g,b, id);
    std::cerr << "AABB drawn? '" << res << "'" << std::endl;
    viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
}


/*
Displays a wireframe OBB from relative 'min_point' to relative 'max_point', at position 'pos' with rotation 'rot'
*/
void drawOBB(pcl::visualization::PCLVisualizer viewer, pcl::PointXYZ min_point, pcl::PointXYZ max_point, pcl::PointXYZ pos, Eigen::Matrix3f rot, double r, double g, double b, std::string id) {
    Eigen::Vector3f position (pos.x, pos.y, pos.z);
    Eigen::Quaternionf rot_quat (rot);
    position = pos.getArray3fMap();
    bool res = viewer.addCube (position, rot_quat, max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z, id);
    std::cerr << "OBB drawn? '" << res << "'" << std::endl;
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, id);
}

void drawOBB(pcl::visualization::PCLVisualizer::Ptr viewer_ptr, pcl::PointXYZ min_point, pcl::PointXYZ max_point, pcl::PointXYZ pos, Eigen::Matrix3f rot, double r, double g, double b, std::string id) {
    Eigen::Vector3f position (pos.x, pos.y, pos.z);
    Eigen::Quaternionf rot_quat (rot);
    position = pos.getArray3fMap();
    bool res = viewer_ptr->addCube (position, rot_quat, max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z, id);
    std::cerr << "OBB drawn? '" << res << "'" << std::endl;
    viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
    viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, id);
}


/**
 * Displays given text in the bottom left corner, automatically placing it above text previously placed using this function
*/
void displayText(pcl::visualization::PCLVisualizer::Ptr viewer, const std::string &text) {
    viewer->addText(text, 0, line_default_offset + line_counter * line_height);
    std::cerr << line_counter << std::endl;
    line_counter++;
}