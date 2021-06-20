# Roadpointsextraction
This code aims to extract point that belongs to the planar surface (road points) of the point cloud, the workflow starts by filtering the data to reduce noise and outliers. Secondly, the data is segmented using RANSAC algorithm and thereafter is enhanced by the Growing Region approach. Lastly, the planar points and the surrounding objects (clusters) are visualised.

Dependencies: 
--PCL 1.11.1
