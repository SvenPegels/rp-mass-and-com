
import matplotlib.pyplot as plt
import pandas as pd
import re
import os

DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/volume_full_mesh/results_all_11_48.csv"
OUTPUT_PATH = "/home/svenp/rp-mass-and-com/test_results"

ESTIMATION_INPUT_FILE_SUFFIX = ".ply"


OBJECTS = ['sphere', 'cube', 'cylinder', 'cone', 'torus', 'mug']
COLUMNS = ['file_name', 'aabb_volume_relative', 'obb_volume_relative', 'chull_volume_relative']


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
    actual = df.at[i, 'actual_volume']

    aabb_vol = df.at[i, 'aabb_volume']
    aabb_rel = aabb_vol / actual
    df.at[i, 'aabb_volume_relative'] = aabb_rel

    obb_vol = df.at[i, 'obb_volume']
    obb_rel = obb_vol / actual
    df.at[i, 'obb_volume_relative'] = obb_rel

    chull_vol = df.at[i, 'chull_volume']
    chull_rel = chull_vol / actual
    df.at[i, 'chull_volume_relative'] = chull_rel


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