#!/bin/bash
#echo "Manipulation AND Onbject Recognition"

echo "Execute ROSCORE" 
xterm -e "roscore;	exec bash" &

sleep 8s

echo "execute openni2_launch" 
xterm -e "roslaunch openni2_launch openni2.launch;	exec bash" &

sleep 5s

echo "execute KOBUKI_NODE" 
xterm -e "roslaunch kobuki_node minimal.launch;	exec bash" &

sleep 5s

echo "Check camera/driver/depth_registration" 
xterm -e "rosrun rqt_reconfigure rqt_reconfigure;	exec bash" &

sleep 15s

echo "Execute Object Recognition Kitchen linemod detector" 
xterm -e "rosrun object_recognition_core detection -c `rospack find object_recognition_linemod`/conf/detection.ros.ork;	exec bash" &

sleep 5s
  
echo "Execute Object Recognition Kitchen linemod server" &
xterm -e "rosrun object_recognition_ros server -c `rospack find object_recognition_linemod`/conf/detection.ros.ork;	exec bash" &

sleep 5s

echo "Main program" 
xterm -e "roslaunch manip_and_recog manip_and_recog.launch;	exec bash" 

wait
killall roscore
killall rosmaster

