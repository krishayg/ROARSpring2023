from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import mplcursors
with open("ROAR/configurations/major_center_waypoints_1LAPONLY_rightedTEMP.txt","r") as file:
    lines = file.readlines()[0:10000]
points = []
for line in lines:
    parts = line.strip().split(',')
    #if len(parts) >= 0:
    x, y, z = map(float, parts[:3])
    points.append((x, y, z))

# Separate coordinates into x, y, z lists
x = [point[0] for point in points]
y = [point[1] for point in points]
z = [point[2] for point in points]

# Create a figure and a 3D Axes object
fig,ax = plt.subplots()
#ax = fig.add_subplot(111, projection='3d')

# Scatter plot the points
scatter=ax.scatter(x, z, c='r', marker='o')
indices_to_mark_differently = [i for i in range(0, len(points), 1000)]

# Overwrite the specific points to mark them differently
for i in indices_to_mark_differently:
    ax.scatter(x[i], z[i], c='g', marker='o')
    ax.text(x[i], z[i], f"{i}", color='g', fontsize=10)
# Set labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
#ax.set_zlabel('Z')
mplcursors.cursor(scatter).connect("add", lambda sel: sel.annotation.set_text(f"Index: {sel.target.index}"))

# Show the plot
plt.show()