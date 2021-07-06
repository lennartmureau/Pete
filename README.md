# Pete

This ROS package has been written in the first half of 2021 as part of the Robotics and Vision Design minor at the Hague University of Applied Sciences.

The ROS package consists of 6 different nodes. Which can be split into two groups:
- The nodes that create a 2D depth image from incoming pointcloud data.
- The nodes that create a 3D transform of detected objects.

The first group of nodes (consisting of the depth_imager8, depth_imager16 and depth_viewer8) Convert incoming pointcloud data into a 2D image.
The nodes have been written to receive pointcloud data from a Velodyne VLP16 LiDAR using the Velodyne ROS packages https://wiki.ros.org/velodyne.

Both depth_imagers create a grayscale image with an accuracy of 8 or 16 bits respectively. These depth images can be viewed using RViz and are also used by the second group of nodes.
The third node, the depth_viewer8, creates a color based depth image. This image can again be viewed using RViz but does not serve any other purpose than looking interesting for humans.

The second group of nodes (consisting of the object_3D_positioner8, object_3D_positioner16 and tester) can take the image created by the nodes mentioned previously
and combine these with incoming data from detected objects to create a 3D transform to each detected object. For the detection of objects YOLOv4 was with the ROS implementation of darknet
https://github.com/Tossy0423/yolov4-for-darknet_ros. The boundingBoxes messages are used for where the detected objects are located.

The node 'tester' is very usefull for testing (of course) because it does not require darknet to be running, nor does it need to be installed, if this is the case,
the CMake file will need to be changed so that darknet is not a required component and both object_3D_positioner nodes are not compiled.

If you have darknet installed in order to test out one of the object_3D_positioner two small changes need to be made to it before it can operate. The first change is in the
boundingBoxes message type, a uint8 called count needs to be added to the message description. In the YoloObjectDetector.cpp file the following line can than be added between
line 581 and 582: boundingBoxesResults_.count = num; After these changes and having compiled darknet, both object_3D_detectors publish tf2 transform messages for each
detected objects, if the corresponding depth_imager is running as well.

All the nodes have been tested on the NVidia Jetson Xavier AGX running NVidia's version of Ubuntu 18.04
