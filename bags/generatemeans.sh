
source ~/catkin_ws/devel/setup.bash

FILENAME=$1
(rosbag play "depth${FILENAME}cm.bag" &);
(rosrun asus_node asus_node &);


sleep 12;

(rosbag play report.bag &) ; 
(rostopic echo /mean > "mean${FILENAME}cm.txt" &) 
(rostopic echo /stddev > "stddev${FILENAME}cm.txt" &)

sleep 12;

killall rostopic;


sed -n "1~2p" -i'' "mean${FILENAME}cm.txt" 
sed -i'' -e 's/data: //g' "mean${FILENAME}cm.txt"

sed -n "1~2p" -i'' "stddev${FILENAME}cm.txt" 
sed -i'' -e 's/data: //g' "stddev${FILENAME}cm.txt" 
