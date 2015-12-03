
source ~/catkin_ws/devel/setup.bash

APPEND=$1
for sz in "50" "100" ; do #"150" "200" "300"
    
    FILENAME=$sz;
    (rosbag play "depth${FILENAME}cm.bag" &);
    sleep .2;
    (rosrun asus_node asus_node &);


    sleep 14;

    (rosbag play report.bag &) ; 
    (rostopic echo /mean > "mean${FILENAME}cm${APPEND}.txt" &) 
    (rostopic echo /stddev > "stddev${FILENAME}cm${APPEND}.txt" &)

    (rostopic echo /meanG > "meanG${FILENAME}cm${APPEND}.txt" &) 
    (rostopic echo /stddevG > "stddevG${FILENAME}cm${APPEND}.txt" &)
    
    (rostopic echo /meanM > "meanM${FILENAME}cm${APPEND}.txt" &) 
    (rostopic echo /stddevM > "stddevM${FILENAME}cm${APPEND}.txt" &)
    
    (rostopic echo /meanB > "meanB${FILENAME}cm${APPEND}.txt" &) 
    (rostopic echo /stddevB > "stddevB${FILENAME}cm${APPEND}.txt" &)
    
    (rostopic echo /meantA > "meantA${FILENAME}cm${APPEND}.txt" &) 
    (rostopic echo /stddevtA > "stddevtA${FILENAME}cm${APPEND}.txt" &)
    
    (rostopic echo /meantM > "meantM${FILENAME}cm${APPEND}.txt" &) 
    (rostopic echo /stddevtM > "stddevtM${FILENAME}cm${APPEND}.txt" &)

    sleep 14;

    killall rostopic;


    sed -n "1~2p" -i'' "mean${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "mean${FILENAME}cm${APPEND}.txt"

    sed -n "1~2p" -i'' "stddev${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "stddev${FILENAME}cm${APPEND}.txt" 

    
    sed -n "1~2p" -i'' "meanG${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "meanG${FILENAME}cm${APPEND}.txt"

    sed -n "1~2p" -i'' "stddevG${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "stddevG${FILENAME}cm${APPEND}.txt" 

    
    sed -n "1~2p" -i'' "meanM${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "meanM${FILENAME}cm${APPEND}.txt"

    sed -n "1~2p" -i'' "stddevM${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "stddevM${FILENAME}cm${APPEND}.txt" 

    
    sed -n "1~2p" -i'' "meanB${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "meanB${FILENAME}cm${APPEND}.txt"

    sed -n "1~2p" -i'' "stddevB${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "stddevB${FILENAME}cm${APPEND}.txt" 


    
    sed -n "1~2p" -i'' "meantA${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "meantA${FILENAME}cm${APPEND}.txt"

    sed -n "1~2p" -i'' "stddevtA${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "stddevtA${FILENAME}cm${APPEND}.txt" 


    
    sed -n "1~2p" -i'' "meantM${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "meantM${FILENAME}cm${APPEND}.txt"

    sed -n "1~2p" -i'' "stddevtM${FILENAME}cm${APPEND}.txt" 
    sed -i'' -e 's/data: //g' "stddevtM${FILENAME}cm${APPEND}.txt" 

done
