
source ~/catkin_ws/devel/setup.bash

APPEND=$1
for sz in "50" "100" "150" "200" "300"; do
    
    FILENAME=$sz;
    (rosbag play "depth${FILENAME}cm.bag" &);
    sleep .5;
    (rosrun asus_node asus_node &);


    sleep 14;

    (rosbag play report.bag &) ; 
    (rostopic echo /mean > "mean${FILENAME}cm${APPEND}.txt" &) 
    (rostopic echo /stddev > "stddev${FILENAME}cm${APPEND}.txt" &)

    sleep 14;

    killall rostopic;


    sed -n "1~2p" -i'' "mean${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "mean${FILENAME}cm${APPEND}.txt"

    sed -n "1~2p" -i'' "stddev${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "stddev${FILENAME}cm${APPEND}.txt" 

done
