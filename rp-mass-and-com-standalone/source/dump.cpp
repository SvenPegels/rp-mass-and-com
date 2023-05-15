

/**
 * Bounding box calculation
 * //TODO: Add rotation
*/
// void drawWireframeBox(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointXYZ min, pcl::PointXYZ max, pcl::PointXYZ position) {
//     counter++;

//     pcl::PointXYZ p_top_1 (pcl::PointXYZ(position.x + min.x, position.y + min.y, position.z + min.z));
//     pcl::PointXYZ p_top_2 (pcl::PointXYZ(position.x + min.x, position.y + min.y, position.z + max.z));
//     pcl::PointXYZ p_top_3 (pcl::PointXYZ(position.x + max.x, position.y + min.y, position.z + max.z));
//     pcl::PointXYZ p_top_4 (pcl::PointXYZ(position.x + max.x, position.y + min.y, position.z + min.z));

//     pcl::PointXYZ p_bottom_1 (pcl::PointXYZ(position.x + min.x, position.y + max.y, position.z + min.z));
//     pcl::PointXYZ p_bottom_2 (pcl::PointXYZ(position.x + min.x, position.y + max.y, position.z + max.z));
//     pcl::PointXYZ p_bottom_3 (pcl::PointXYZ(position.x + max.x, position.y + max.y, position.z + max.z));
//     pcl::PointXYZ p_bottom_4 (pcl::PointXYZ(position.x + max.x, position.y + max.y, position.z + min.z));
//     // Draw top square
//     viewer->addLine(p_top_1, p_top_2, "top12_" + counter);
//     viewer->addLine(p_top_2, p_top_3, "top23_" + counter);
//     viewer->addLine(p_top_3, p_top_4, "top34_" + counter);
//     viewer->addLine(p_top_4, p_top_1, "top41_" + counter);

//     // Draw bottom square
//     viewer->addLine(p_bottom_1, p_bottom_2, "bottom12_" + counter);
//     viewer->addLine(p_bottom_2, p_bottom_3, "bottom23_" + counter);
//     viewer->addLine(p_bottom_3, p_bottom_4, "bottom34_" + counter);
//     viewer->addLine(p_bottom_4, p_bottom_1, "bottom41_" + counter);

//     // Draw side lines between
//     viewer->addLine(p_top_1, p_bottom_1, "side11_" + counter);
//     viewer->addLine(p_top_2, p_bottom_2, "side22_" + counter);
//     viewer->addLine(p_top_3, p_bottom_3, "side33_" + counter);
//     viewer->addLine(p_top_4, p_bottom_4, "side44_" + counter);
// }

// void drawWireframeBox(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointXYZ min, pcl::PointXYZ max, pcl::PointXYZ position, Eigen::Matrix3f rotation) {
//     counter++;

//     // TODO: Look into (https://pcl.readthedocs.io/projects/tutorials/en/latest/moment_of_inertia.html?highlight=lamppost) :
//     /*
//     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//     Eigen::Quaternionf quat (rotational_matrix_OBB);
//     viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
//     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
//     */


//     // Eigen::Vector3f position (position.x, position.y, position.z);
//     // Eigen::Quaternionf quat (rotation);
//     // // viewer->addCube (position, quat, max.x - min.x, max.y - min.y, max.z - min.z, "OBB");
//     // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");


//     // pcl::PointXYZ p_top_1 (pcl::PointXYZ(position.x + (min).x, position.y + min.y, position.z + min.z));
//     // pcl::PointXYZ p_top_2 (pcl::PointXYZ(position.x + min.x, position.y + min.y, position.z + max.z));
//     // pcl::PointXYZ p_top_3 (pcl::PointXYZ(position.x + max.x, position.y + min.y, position.z + max.z));
//     // pcl::PointXYZ p_top_4 (pcl::PointXYZ(position.x + max.x, position.y + min.y, position.z + min.z));

//     // pcl::PointXYZ p_bottom_1 (pcl::PointXYZ(position.x + min.x, position.y + max.y, position.z + min.z));
//     // pcl::PointXYZ p_bottom_2 (pcl::PointXYZ(position.x + min.x, position.y + max.y, position.z + max.z));
//     // pcl::PointXYZ p_bottom_3 (pcl::PointXYZ(position.x + max.x, position.y + max.y, position.z + max.z));
//     // pcl::PointXYZ p_bottom_4 (pcl::PointXYZ(position.x + max.x, position.y + max.y, position.z + min.z));
//     // // Draw top square
//     // viewer->addLine(p_top_1, p_top_2, "top12_" + counter);
//     // viewer->addLine(p_top_2, p_top_3, "top23_" + counter);
//     // viewer->addLine(p_top_3, p_top_4, "top34_" + counter);
//     // viewer->addLine(p_top_4, p_top_1, "top41_" + counter);

//     // // Draw bottom square
//     // viewer->addLine(p_bottom_1, p_bottom_2, "bottom12_" + counter);
//     // viewer->addLine(p_bottom_2, p_bottom_3, "bottom23_" + counter);
//     // viewer->addLine(p_bottom_3, p_bottom_4, "bottom34_" + counter);
//     // viewer->addLine(p_bottom_4, p_bottom_1, "bottom41_" + counter);

//     // // Draw side lines between
//     // viewer->addLine(p_top_1, p_bottom_1, "side11_" + counter);
//     // viewer->addLine(p_top_2, p_bottom_2, "side22_" + counter);
//     // viewer->addLine(p_top_3, p_bottom_3, "side33_" + counter);
//     // viewer->addLine(p_top_4, p_bottom_4, "side44_" + counter);
// }


// /*
// Calculate Axis Aligned Bounding Box (AABB)
// */
// void calcAABBfromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
//     pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
//     feature_extractor.setInputCloud(cloud_ptr);
//     feature_extractor.compute();

//     pcl::PointXYZ min_point_AABB;
//     pcl::PointXYZ max_point_AABB;

//     feature_extractor.getAABB(min_point_AABB, max_point_AABB);

//     float volume = (max_point_AABB.x - min_point_AABB.x)*(max_point_AABB.y - min_point_AABB.y)*(max_point_AABB.z - min_point_AABB.z);
//     std::cerr << "AABB from (" 
//         << min_point_AABB.x << "," << min_point_AABB.y << "," << min_point_AABB.z << ") to ("
//         << max_point_AABB.x << "," << max_point_AABB.y << "," << max_point_AABB.z << ") to (" << std::endl;
//     std::cerr << "   " << "- volume: " << volume << std::endl;
// }

// /*
// Estimate Oriented Bounding Box (OBB)
// */
// void estimateOBBromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
//     pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
//     feature_extractor.setInputCloud(cloud_ptr);
//     feature_extractor.compute();

//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;

//     feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

//     // int volume = (max_point_OBB.x - min_point_AABB.x)*(max_point_AABB.y - min_point_AABB.y)*(max_point_AABB.z - min_point_AABB.z);
//     std::cerr << "OOB from (" 
//         << min_point_OBB.x << "," << min_point_OBB.y << "," << min_point_OBB.z << ") to ("
//         << max_point_OBB.x << "," << max_point_OBB.y << "," << max_point_OBB.z << ") to (" << std::endl;
//     // std::cerr << "   " << "- volume: " << volume << std::endl;
// }

// std::string PointXYZtoString(pcl::PointXYZ p) {
//     return "(" + std::to_string(p.x) + "," + std::to_string(p.y) + "," + std::to_string(p.z) + ")";
// }


/**
 * Visualization
*/

void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    // viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 0.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    // user_data++;
}