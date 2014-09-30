#!/bin/bash
aria() {
    if [ $1 = "echo" ]; then
        if [ -z "$3" ]; then
            rostopic echo "/currentor_socket/sensor_array/"$2"/data"
        else
            rostopic echo "/currentor_socket/sensor_array/"$2"/data"[$3]
        fi
    elif [ $1 = "once" ]; then
        if [ -z "$3" ]; then
            rostopic echo -n1 "/currentor_socket/sensor_array/"$2"/data"
        else
            rostopic echo -n1 "/currentor_socket/sensor_array/"$2"/data"[$3]
        fi      
    elif [ $1 = "pub" ]; then
        rostopic pub --once "/ros2http/socket_listener/json_string" std_msgs/String '{data: "{\"method\":\"'$2'\",\"params\":\"'$3'\",\"id\":\"0\"}"}'
    else
        rostopic pub --once "/aria/commandline" std_msgs/String $1
    fi
}


val=$(aria once $1 $2)
if [ "$(echo $val)" == "$(echo $3' ---')" ]; then
    exit 1
else
    exit 0
fi