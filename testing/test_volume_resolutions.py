import os
import re

UNITY_EXECUTABLE = "~/Unity/Hub/Editor/2021.3.23f1/Editor/Unity -projectPath ~/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/ -executeMethod ResolutionPCDGenerator.GenerateResolutions"
# UNITY_OUTPUT_DIR = "/home/svenp/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/PCD_TEST_RES" # When changed, also change in the Unity script
# UNITY_OUTPUT_DIR = "/home/svenp/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/PCD_TEST_RES_TEMP/cube_70deg_-30_30_-30_16-1920"
# UNITY_OUTPUT_DIR = "/home/svenp/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/PCD_TEST_RES_TEMP/sphere_57deg_-30_30_-30_16-1920"
# UNITY_OUTPUT_DIR = "/home/svenp/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/PCD_TEST_RES_TEMP/mug_57deg_-30_30_-30_16-1920"
# UNITY_OUTPUT_DIR = "/home/svenp/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/PCD_TEST_RES_TEMP/all_57deg_-30_30_-30_16-1920"
UNITY_OUTPUT_DIR = "/home/svenp/rp-mass-and-com/rp-mass-and-com-depth-camera-victoria/My-project/PCD_TEST_RES_TEMP/all_16by9_43degvert_-30_30_-30_16-1920"

COPY_FILE_MODIFIER = "*.pcd"

ESTIMATION_EXECUTABLE = "/home/svenp/rp-mass-and-com/rp-mass-and-com-standalone/build/./volume_estimation_pcd"
ESTIMATION_INPUT_DIR = "/home/svenp/rp-mass-and-com/pcd_files/resolution_generated_clouds"
ESTIMATION_DATA_INPUT_DIR = "/home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/data"
# INPUT_DIR = "/home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/high_quality/ply" NOTE THIS LINE IS WRONG
# DATA_INPUT_DIR = "/home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/high_quality/data"
ESTIMATION_INPUT_FILE_SUFFIX = ".pcd"
ESTIMATION_DATA_FILE_SUFFIX = "_data.txt"
ESTIMATION_RESULTS_OUTPUT_DIR = "/home/svenp/rp-mass-and-com/test_results/volume_resolutions"


"""
Partial view generation
"""
print("Testing - Generating partial views in Unity...")
# os.system(UNITY_EXECUTABLE)
print("Testing - Done!")


"""
File Copying
"""
print("Testing - Copying generated .pcd files from '" + UNITY_OUTPUT_DIR + "' to '" + ESTIMATION_INPUT_DIR + "'...")
os.system("cp " + UNITY_OUTPUT_DIR + "/" + COPY_FILE_MODIFIER + " " + ESTIMATION_INPUT_DIR)
print("Testing - Done!")


"""
Volume estimation
"""
print("Testing - Estimating volumes...")
# Create results file containing data headers from the template file, for the estimation to write results to. w = overwrite contents.
f_template = open(ESTIMATION_RESULTS_OUTPUT_DIR + "/results_template.csv", "r")
res_template = f_template.read()

f_results = open(ESTIMATION_RESULTS_OUTPUT_DIR + "/results.csv", "w")
# f_results_results.write("file_name,actual_volume,aabb_volume,obb_volume,tetra_volume,chull_volume")
f_results.write(res_template)
f_results.close()

for file_name in os.listdir(ESTIMATION_INPUT_DIR):
    file_path = os.path.join(ESTIMATION_INPUT_DIR, file_name)
    # checking if it is a file ending with INPUT_FILE_SUFFIX
    if os.path.isfile(file_path) and file_path.endswith(ESTIMATION_INPUT_FILE_SUFFIX):

        data_file_name = file_name[:-len(ESTIMATION_INPUT_FILE_SUFFIX)] + ESTIMATION_DATA_FILE_SUFFIX # Trim the INPUT_FILE_SUFFIX
        data_file_name = re.sub('_res_[0-9]+', '', data_file_name) # Trim the 'resolution suffix'. Works up to res_99.
        
        data_file_path = os.path.join(ESTIMATION_DATA_INPUT_DIR, data_file_name)

        command = ESTIMATION_EXECUTABLE + " true " + file_path + " " + data_file_path + " " + ESTIMATION_RESULTS_OUTPUT_DIR
        os.system(command)

        # break
print("Testing - Done!")


"""
Folder cleanup
"""
print("Testing - Cleaning up folders '" + UNITY_OUTPUT_DIR + "' and '" + ESTIMATION_INPUT_DIR + "'...")
# os.system("rm " + UNITY_OUTPUT_DIR + "/*.pcd")
os.system("rm " + ESTIMATION_INPUT_DIR + "/*.pcd")
print("Testing - Done!")
