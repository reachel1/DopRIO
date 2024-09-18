import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np

# Read the point cloud file
pcd = o3d.io.read_point_cloud("ntu_loop3_0.pcd")

# Convert the point cloud to NumPy data
points = np.asarray(pcd.points)

x = points[:,0]
y = points[:,1]
z = points[:,2]

# Save the image as high DPI image
fig = plt.figure()
fig.set_dpi(400)
fig.patch.set_facecolor('black')
ax = fig.add_subplot(111)

ax.scatter(x,y,c=10*z,cmap='jet',s=0.05, alpha=1,edgecolor='none')

#plt.show()
plt.axis('off')
plt.savefig('3.png', dpi=400, bbox_inches='tight')

