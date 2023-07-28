
import matplotlib.pyplot as plt
import pandas as pd
import re
import os
import math


def ed(x1, y1, z1, x2, y2, z2):
    return math.sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2))

DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/com_full_mesh/results_all_13_51.csv"
OUTPUT_PATH = "/home/svenp/rp-mass-and-com/test_results"

ESTIMATION_INPUT_FILE_SUFFIX = ".ply"


OBJECTS = ['sphere', 'cube', 'cylinder', 'cone', 'torus', 'mug']
COLUMNS = ['file_name', 'aabb_error', 'obb_error', 'chull_avg_error']


df = pd.read_csv(DATA_PATH)
for i, row in df.iterrows():
    # Simplify file name into the object's name (cube/sphere/torus) and resolution's index (0,1,2...)
    # file_name = os.path.basename('/home/svenp/rp-mass-and-com/obj_files/partial_view_base_models/base_quality/ply/mug_triangulated_lc.ply')
    file_name = os.path.basename(df.at[i, 'file_name'])
    # print(file_name)
    new_file_name = file_name[:-len(ESTIMATION_INPUT_FILE_SUFFIX)] # Remove ESTIMATION_INPUT_FILE_SUFFIX
    # print(new_file_name)
    # res_index = int(new_file_name.split("_")[-1])
    new_file_name = new_file_name.split("_", 1)[0]
    # print(new_file_name)
    # print(df.at[i, 'file_name'])
    df.at[i, 'file_name'] = new_file_name
    # df.at[i, 'resolution_index'] = res_index

# Calculate relative error
for i,row in df.iterrows():

    actual_x = df.at[i, 'actual_com_x']
    actual_y = df.at[i, 'actual_com_y']
    actual_z = df.at[i, 'actual_com_z']

    aabb_x = df.at[i, 'aabb_com_x']
    aabb_y = df.at[i, 'aabb_com_y']
    aabb_z = df.at[i, 'aabb_com_z']
    aabb_error = ed(actual_x, actual_y, actual_z, aabb_x, aabb_y, aabb_z)
    df.at[i, 'aabb_error'] = aabb_error

    obb_x = df.at[i, 'obb_com_x']
    obb_y = df.at[i, 'obb_com_y']
    obb_z = df.at[i, 'obb_com_z']
    obb_error = ed(actual_x, actual_y, actual_z, obb_x, obb_y, obb_z)
    df.at[i, 'obb_error'] = obb_error

    # chull_x = df.at[i, 'chull_com_x']
    # chull_y = df.at[i, 'chull_com_y']
    # chull_z = df.at[i, 'chull_com_z']
    # chull_error = ed(actual_x, actual_y, actual_z, chull_x, chull_y, chull_z)
    # df.at[i, 'chull_error'] = chull_error

    chull_avg_x = df.at[i, 'chull_average_x']
    chull_avg_y = df.at[i, 'chull_average_y']
    chull_avg_z = df.at[i, 'chull_average_z']
    chull_avg_error = ed(actual_x, actual_y, actual_z, chull_avg_x, chull_avg_y, chull_avg_z)
    df.at[i, 'chull_avg_error'] = chull_avg_error


table_string = ""
table_string += '\\hline\n'

for col in COLUMNS:
    table_string += col
    if COLUMNS[-1] != col:
        table_string += ' & '
    else:
        table_string += ' \\\\'
table_string += '\n'

table_string += '\\hline \\hline\n'

for o in OBJECTS:
    row = df.loc[df['file_name'] == o]
    row.reset_index()
    # print(row)
    for col in COLUMNS:
        val = row.iloc[0][col]
        if col == 'file_name':
            val = val.capitalize()
        
        # table_string += str(val)
        if isinstance(val, str):
            table_string += val
        else:
            table_string += "{:.2f}".format(val)
        
        if COLUMNS[-1] != col:
            table_string += ' & '
        else:
            table_string += ' \\\\'

    table_string += '\n'
    table_string += '\\hline\n'


print(table_string)