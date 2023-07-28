#!/bin/sh
# This is a comment!

echo Generating partial views in Unity...
~/Unity/Hub/Editor/2021.3.23f1/Editor/Unity -projectPath ~/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/ -executeMethod PartialViewPCDGenerator.Generate
echo Done!

echo Moving .pcd files from \"My-project/PCD\" to \"rp-mass-and-com/pcd_files/partial_view_generated_clouds\"
# If the 'input' folder is empty it will throw an error: "No such file or directory"
cp ~/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/PCD/*.pcd ~/rp-mass-and-com/pcd_files/partial_view_generated_clouds
echo Done!

# echo Estimating volumes...
# cd ~/rp-mass-and-com/rp-mass-and-com-standalone/build
# ./volume_estimation_pcd ~/rp-mass-and-com/pcd_files/partial_view_generated_clouds/torus_triangulated_lc_cam_2.pcd ~/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/data/torus_triangulated_lc_data.txt
# echo Done!
