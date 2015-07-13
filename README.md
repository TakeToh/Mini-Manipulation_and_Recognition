#ROBOCUP2015 STAGE1 Manipulation&Object Recognition
##How to use -run each other-

###roslaunch openni2_launch openni2.launch
###roslaunch kobuki_node minimal.launch
###rosrun rqt_reconfigure rqt_reconfigure
Select /camera/driver from the drop-down menu and enable the depth_registration checkbox. In RViz, change the PointCloud2 topic to /camera/depth_registered/points and set the Color Transformer to RGB8 to see both color and 3D point cloud of your scene. The detailed explanation can be found here:

###rosrun object_recognition_core detection -c `rospack find object_recognition_linemod`/conf/detection.ros.ork
###rosrun object_recognition_ros server -c `rospack find object_recognition_linemod`/conf/detection.ros.ork

###roslaunch manip_and_recog manip_and_recog.launch

##How to use -using scriptr-
###roscd manip_and_recog
###./execute.sh

## Needed Package

### Object Recognition Kitche install
http://wg-perception.github.io/object_recognition_core/install.html#install
