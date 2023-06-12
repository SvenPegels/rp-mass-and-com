# Want a
import matplotlib.pyplot as plt

x = [0,1,2,3]
x_labels = [16,80,160, 640]
y_cone = [204,1187,1195,1291]
y_actual=[2080,2080,2080,2080]

plt.plot(x,y_cone, label="cone")
plt.plot(x,y_actual, label="actual")

plt.xticks(x, x_labels)

plt.xlabel("width component of the resolution (pixels) (16:9 aspect ratio)")
plt.ylabel("error (%)")

plt.show()