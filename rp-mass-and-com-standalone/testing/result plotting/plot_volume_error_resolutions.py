# Not used
import matplotlib.pyplot as plt
import pandas as pd
import re
import os


DATA_PATH = "/home/svenp/rp-mass-and-com/test_results/volume_resolutions/results_1.csv"
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
        ,(640, 360)]
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
#         ,(320, 180)
#         ,(640, 360)
#         ,(960, 540)
#         ,(1280, 720)
#         ,(1920, 1080)]

ESTIMATION_INPUT_FILE_SUFFIX = ".pcd"

OBJECT = 'sphere'


df = pd.read_csv(DATA_PATH)
# print(df.head(5).to_string())
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

    aabb_error = abs(df.at[i, 'aabb_volume'] - actual)
    aabb_error_p = aabb_error / actual * 100
    df.at[i, 'aabb_error'] = aabb_error
    df.at[i, 'aabb_error_p'] = aabb_error_p

    obb_error = abs(df.at[i, 'obb_volume'] - actual)
    obb_error_p = obb_error / actual * 100
    df.at[i, 'obb_error'] = obb_error
    df.at[i, 'obb_error_p'] = obb_error_p

    chull_error = abs(df.at[i, 'chull_volume'] * 2 - actual)
    chull_error_p = chull_error / actual * 100
    df.at[i, 'chull_error'] = chull_error
    df.at[i, 'chull_error_p'] = chull_error_p


df = df.sort_values(by=['file_name', 'resolution_index'])

grouped = df.groupby(['file_name'])
grouped_list = grouped.apply(list)
# print(grouped)

# print(grouped.indices)
# for key, value in grouped.indices.items():
#     print(key)
#     print(value)

objects = grouped.indices.keys()

# print(df.head(5).to_string())
# print(df.to_string())

# Extract data to plot
x_labels = [str(res[0]) + "x" + str(res[1]) for res in resolutions]
x = range(len(x_labels))

# y_cone = grouped.get_group('cone')['chull_volume'].tolist()
# y_cone_factor = [i*2 for i in y_cone]
# y_cone_actual = [grouped.get_group('cone')['actual_volume'].tolist()[0] for i in range(len(y_cone))]

y_sphere_chull = grouped.get_group(OBJECT)['chull_error_p'].tolist()
# y_sphere_chull_factor = [i*2 for i in y_sphere_chull]

y_sphere_aabb = grouped.get_group(OBJECT)['aabb_error_p'].tolist()

y_sphere_obb = grouped.get_group(OBJECT)['obb_error_p'].tolist()

y_sphere_actual = grouped.get_group(OBJECT)['actual_volume'].tolist()
print(y_sphere_actual[0])


print()


# Plot data

# plt.plot(x,y_cone_factor, label="cone")
# plt.plot(x,y_cone_actual, label="cone actual")

# plt.plot(x,y_sphere_actual, label="actual")
plt.plot(x,y_sphere_chull, label="chull (factor=2)")
plt.plot(x,y_sphere_aabb, label="AABB")
plt.plot(x,y_sphere_obb, label="OBB")

plt.xticks(x, x_labels, rotation=-30)
plt.ylim(0, 100)

plt.xlabel("Resolution (pixels)")
plt.ylabel("Error (%)")

plt.legend()
if len(OBJECT) > 0:
    plt.title('Center of Mass estimates of a ' + OBJECT)
else:
    plt.title('Center of Mass estimates')

plt.tight_layout()

plt.show()


