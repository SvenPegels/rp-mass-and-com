# Is used
import matplotlib.pyplot as plt
import pandas as pd
import re
import os
import numpy as np


# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": ""
# })


# DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/volume_resolutions/results_1.csv"
# DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/volume_resolutions/results_sphere_57deg_-30_30_-30_16-1920.csv"
# OBJECT = 'sphere'
# DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/volume_resolutions/results_cube_70deg_-30_30_-30_16-1920.csv"
# OBJECT = 'cube'
# DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/volume_resolutions/results_mug_57deg_-30_30_-30_16-1920.csv"
# OBJECT = 'mug'
DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/volume_resolutions/results_all_57deg_-30_30_-30_16-1920.csv"
DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/volume_resolutions/results_all_w_size_57deg_-30_30_-30_16-1920.csv"
OBJECT = 'mug'
OBJECTS = ['mug', 'cube', 'sphere', 'cone', 'torus', 'cylinder']
# OBJECTS = ['mug', 'cube', 'sphere']
# OBJECTS = ['sphere']


# resolutions = [(16, 9)
#         ,(32, 18)
#         ,(48, 27)
#         ,(64, 36)
#         ,(80, 45)
#         ,(96, 54)
#         ,(112, 63)
#         ,(128, 72)
#         ,(144, 81)
#         ,(160, 90) 
#         ,(640, 360)]
resolutions = [(16, 9)
        ,(32, 18)
        ,(48, 27)
        ,(64, 36)
        ,(80, 45)
        ,(96, 54)
        ,(112, 63)
        ,(128, 72)
        ,(144, 81)
        ,(160, 90)
        ,(320, 180)
        ,(640, 360)
        ,(960, 540)
        ,(1280, 720)
        ,(1920, 1080)]

ESTIMATION_INPUT_FILE_SUFFIX = ".pcd"



df = pd.read_csv(DATA_PATH)
print(df.head(5).to_string())
# Split file_name into object's name and resolution index
for i, row in df.iterrows():
    # Simplify file name into the object's name (cube/sphere/torus) and resolution's index (0,1,2...)
    file_name = os.path.basename(df.at[i, 'file_name'])
    new_file_name = file_name[:-len(ESTIMATION_INPUT_FILE_SUFFIX)] # Remove ESTIMATION_INPUT_FILE_SUFFIX
    res_index = int(new_file_name.split("_")[-1])
    new_file_name = new_file_name.split("_", 1)[0]
    df.at[i, 'file_name'] = new_file_name
    df.at[i, 'resolution_index'] = res_index

# Calculate error and error percentage
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


df = df.sort_values(by=['file_name', 'resolution_index'])
print(df.to_string())

grouped = df.groupby(['file_name'])
grouped_list = grouped.apply(list)
print(grouped)

print(grouped.indices)
for key, value in grouped.indices.items():
    print(key)
    print(value)

# objects = grouped.indices.keys()

# print(df.head(5).to_string())
# print(df.to_string())

# Extract data to plot
x_labels = [str(res[0]) + "x" + str(res[1]) for res in resolutions]
x = range(len(x_labels))
plt.xticks(x, x_labels, rotation=-35)
# plt.yticks(np.arange(0, max(max(y_sphere_chull_factor), max(y_sphere_aabb), max(y_sphere_obb), max(y_sphere_actual)), 0.2))

# plt.figure(figsize=(5, 3.5))


for o in OBJECTS:

    y_actual = grouped.get_group(o)['actual_volume'].tolist()[0]

    # Gather data
    y_chull = grouped.get_group(o)['chull_volume'].tolist()
    y_chull_rel = grouped.get_group(o)['chull_volume_relative'].tolist()
    y_chull_rel_factor = [i*2 for i in y_chull_rel]
    y_chull_normalised_max = [i/max(y_chull_rel) for i in y_chull_rel]
    y_chull_normalised_last = [i/y_chull_rel[-1] for i in y_chull_rel]

    y_aabb = grouped.get_group(o)['aabb_volume'].tolist()
    y_aabb_rel = grouped.get_group(o)['aabb_volume_relative'].tolist()
    y_aabb_normalised_max = [i/max(y_aabb_rel) for i in y_aabb_rel]
    y_aabb_normalised_last = [i/y_actual for i in y_aabb_rel]

    y_obb = grouped.get_group(o)['obb_volume'].tolist()
    y_obb_rel = grouped.get_group(o)['obb_volume_relative'].tolist()
    y_obb_normalised_max = [i/max(y_obb_rel) for i in y_obb_rel]
    y_obb_normalised_last = [i/y_actual for i in y_obb_rel]

    # y_cloud_size = grouped.get_group(o)['cloud_size'].tolist()
    # y_cloud_size_max = max(y_cloud_size)
    # y_cloud_size_normalized = [float(i)/float(y_cloud_size_max) for i in y_cloud_size]

    y_actual = [1 for i in range(len(x))]
    print(y_actual[0])

    # Plot data
    # Give all lines for the same object the same color, and each method a different line style

    # Normalised
    # p = plt.plot(x,y_chull_normalised_last, label=o, marker="+", markersize=8, linestyle='solid')
    # plt.plot(x,y_aabb_normalised_last, marker="+", markersize=8, linestyle='dotted', color=p[-1].get_color())
    # plt.plot(x,y_obb_normalised_last, marker="+", markersize=8, linestyle='dashed', color=p[-1].get_color())

    # Relative
    p = plt.plot(x,y_chull_rel, label=o, marker="+", markersize=8, linestyle='solid')
    plt.plot(x,y_aabb_rel, marker="+", markersize=8, linestyle='dotted', color=p[-1].get_color())
    plt.plot(x,y_obb_rel, marker="+", markersize=8, linestyle='dashed', color=p[-1].get_color())


plt.plot([], [], label="Convex Hull", linestyle='solid', color='black')
plt.plot([], [], label=" AABB", linestyle='dotted', color='black')
plt.plot([], [], label="OBB", linestyle='dashed', color='black')

# plt.plot(y_actual, color="black", marker="|")

plt.grid(axis="both")
plt.ylim(bottom=0) # Does this make sense?
# plt.ylim(0, 3)

plt.xlabel("Resolution (pixels)")
plt.ylabel("Relative volume estimate")

plt.legend()
if len(OBJECTS) > 1:
    plt.title('Volume estimates')
else:
    plt.title('Volume estimates of a ' + OBJECT)

plt.tight_layout()

plt.show()


