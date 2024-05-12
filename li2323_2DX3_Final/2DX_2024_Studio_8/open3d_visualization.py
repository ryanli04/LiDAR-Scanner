import numpy as np
import open3d as o3d

steps = 32                  #num of scans per rotation
depth = 7                   #number of rotations
totalScans = steps*depth    #total number of scans

#Read the test data in from the file we created
print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("2dx3data.xyz", format="xyz")

#Lets see what our point cloud data looks like numerically
print("The PCD array:")
print(np.asarray(pcd.points))

#Lets see what our point cloud data looks like graphically
print("Lets visualize the PCD: (spawns seperate interactive window)")
o3d.visualization.draw_geometries([pcd])

#Give each vertex a unique number
yz_slice_vertex = []
for x in range(0,totalScans):
    yz_slice_vertex.append([x])

#Define coordinates to connect lines in each yz slice
lines = []
for x in range(0,totalScans, steps):
    for y in range(31):
        lines.append([yz_slice_vertex[x+y], yz_slice_vertex[x+y+1]])
    lines.append([yz_slice_vertex[x+y+1], yz_slice_vertex[x]])

#Define coordinates to connect lines between current and next yz slice
for x in range(0,(totalScans - steps*2) + 1,steps):
    for y in range(32):
        lines.append([yz_slice_vertex[x+y], yz_slice_vertex[x+y+32]])

#This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically
o3d.visualization.draw_geometries([line_set])