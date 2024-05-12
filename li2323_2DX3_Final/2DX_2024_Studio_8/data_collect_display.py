import serial
import math

ser = serial.Serial('COM#', 115200, timeout = 10)
ser.open

f = open("2dx3data.xyz", "a")
i = 0
x = 0 # Change initial x-displacement (mm)
increment = 600 # x-displacement icrement (mm)
readingData = False
j = 0
steps = 32                  #num of scans per rotation
depth = 3                  #numbeur of rotations
totalScans = steps*depth    #total number of scans
while (j < totalScans):  
    s = ser.readline()
    a = s.decode("utf-8") # Decodes byte input from UART into string 
    a = a[0:-2] # Removes carriage return and newline from string
    if i == 360:
        i = 0
        x = x + increment
    if (a.isdigit() == True):
        readingData = True
        angle = i*math.pi/180 # Obtain angle based on motor rotation
        b = int(a)
        y = b*math.cos(angle) # Calculate y
        z = b*math.sin(angle) # Calcualte z
        f.write('{} {} {}\n'.format(x,y,z)) # Write data to .xyz file 
        i += 11.25
        j+=1
    #if (a.isdigit() == False and readingData == True):
    #    f.close()
    #    f = open("filename.xyz", "a")
    print(a)
f.close()

import numpy as np
import open3d as o3d

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