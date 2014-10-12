function aria {
    if [ $1 = "echo" ] ; then
	if [ -z "$3" ] ; then
	    rostopic echo "/currentor_socket/sensor_array/"$2"/data"
	else
	    rostopic echo "/currentor_socket/sensor_array/"$2"/data"[$3]
	fi
    elif [ $1 = "once" ] ; then
	if [ -z "$3" ] ; then
	    rostopic echo -n1 "/currentor_socket/sensor_array/"$2"/data"
	else
	    rostopic echo -n1 "/currentor_socket/sensor_array/"$2"/data"[$3]
	fi	
    elif [ $1 = "pub" ] ; then
	if [ $2 = "gripper" ] ; then
	    rostopic pub --once "/2ndparty/request/gripper" std_msgs/Float32MultiArray '{data: ['$3','$4','$5']}'
	elif [ $2 = "eye" ] ; then
	    rostopic pub --once "/2ndparty/request/eye" std_msgs/Float32MultiArray '{data: ['$3','$4','$5']}'
	elif [ $2 = "feedback" ] ; then
	    mode=0
	    if [ $3 = "velocity" ] ; then
		mode=0
	    elif [ $3 = "torque" ] ; then
		mode=1
	    elif [ $3 = "position" ] ; then
		mode=2
	    elif [ $3 = "Kp" ] ; then
		mode=3
	    elif [ $3 = "Kd" ] ; then
		mode=4
	    elif [ $3 = "Ct" ] ; then
		mode=5
	    elif [ $3 = "Cp" ] ; then
		mode=6
	    else
		mode=0
	    fi
	    rostopic pub --once "/ros2http/socket_listener/json_string" std_msgs/String '{data: "{\"method\":\"setFeedback\",\"params\":\"['$mode']\",\"id\":\"0\"}"}'
	else
	    rostopic pub --once "/ros2http/socket_listener/json_string" std_msgs/String '{data: "{\"method\":\"'$2'\",\"params\":\"'$3'\",\"id\":\"0\"}"}'
	fi
    else
	rostopic pub --once "/aria/commandline" std_msgs/String $1
    fi
}

function ariacore {
    okay=0
    while [ $okay -lt 19 ]
    do
	aria pub sendZero
	sleep 3s
	okay=0
	for i in {1..20}
	do
	    pos=$(aria once voltage $i)
	    if [ "$(echo $pos)" == "1.0 ---" ]; then
		echo $i" "$pos
		okay=$((okay+1))
	    else
		echo -e "\e[1;31m$i missing\e[m"
		okay=$((okay+0))
	    fi
	done
    done
    aria apply_gain
    sleep 3s
    okay=0
    while [ $okay -lt 19 ]
    do
    	aria apply_gain
    	sleep 3s
    	okay=0
    	echo 'please make sure feedback is set to Kp! to proceed'
    	for i in {1..20}
    	do
    	    kp=$(aria once velocity $i)
    	    if [ "$(echo $kp)" == "0.0 ---" ]; then
		echo -e "\e[1;31m$i missing\e[m"
    		okay=$((okay+0))
    	    else
    		echo $i" "$kp
    		okay=$((okay+1))
    	    fi
    	done
    done
    okay=0
    while [ $okay -lt 19 ]
    do
	aria initiate
	sleep 5s
	okay=0
	for i in {1..20}
	do
	    mode=$(aria once voltage $i)
	    if [ "$(echo $mode)" == "2.0 ---" ]; then
		echo $i" "$mode
		okay=$((okay+1))
	    else
		echo -e "\e[1;31m$i missing\e[m"
		okay=$((okay+0))
	    fi
	done
    done
}