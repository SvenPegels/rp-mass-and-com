import os

EXECUTABLE = "/home/svenp/rp-mass-and-com/rp-mass-and-com-standalone/build/./center_of_mass_estimation_obj"
# INPUT_DIR = "/home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/ply"
# DATA_INPUT_DIR = "/home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/data"
INPUT_DIR = "/home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/high_quality/ply"
DATA_INPUT_DIR = "/home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/high_quality/data"
INPUT_FILE_SUFFIX = ".ply"
DATA_FILE_SUFFIX = "_data.txt"
RESULTS_OUTPUT_DIR = "/home/svenp/rp-mass-and-com/test_results/com_full_mesh"


# Create results file containing data headers from the template file, for the estimation to write results to. w = overwrite contents.
f_template = open(RESULTS_OUTPUT_DIR + "/results_template.csv", "r")
res_template = f_template.read()

f_results = open(RESULTS_OUTPUT_DIR + "/results.csv", "w")
f_results.write(res_template)
f_results.close()


for file_name in os.listdir(INPUT_DIR):
    file_path = os.path.join(INPUT_DIR, file_name)
    # checking if it is a file ending with INPUT_FILE_SUFFIX
    if os.path.isfile(file_path) and file_path.endswith(INPUT_FILE_SUFFIX):
        data_file_name = file_name[:-len(INPUT_FILE_SUFFIX)] + DATA_FILE_SUFFIX # Trim the INPUT_FILE_SUFFIX
        data_file_path = os.path.join(DATA_INPUT_DIR, data_file_name)

        command = EXECUTABLE + " true " + file_path + " " + data_file_path + " " + RESULTS_OUTPUT_DIR
        os.system(command)

        # break