#!/usr/bin/env sh

DEF_CMD="rostopic pub -r 10 /invented_pendulum/torque_offset_vector std_msgs/Float32MultiArray";

while [ "true" ];
do
    echo "[command line controller]"
    echo -e "\t i: forward";
    echo -e "\t j: left";
    echo -e "\t k: backward";
    echo -e "\t l: right";
    read CMD;
    if [ "$CMD" == "i" ];
    then
	CMD="$DEF_CMD \"{data: [-1.0, 1.0]}\" &";
    elif [ "$CMD" == "j" ]
    then
	CMD="$DEF_CMD \"{data: [1.0, 1.0]}\" &";
    elif [ "$CMD" == "k" ]
    then
	CMD="$DEF_CMD \"{data: [1.0, -1.0]}\" &";
    elif [ "$CMD" == "l" ]
    then
	CMD="$DEF_CMD \"{data: [-1.0, -1.0]}\" &";
    else
	CMD="$DEF_CMD \"{data: [0.0, 0.0]}\" &";
    fi;
    if [ "$CMD" ];
    then
	for PRS in `ps aux | grep "rostopic pub -r 10 /invented_pendulum/torque_offset_vector" | grep -v grep | sed "s/\ /@/g"`;
	do
	    kill -2 `echo $PRS | sed "s/\@\+/\\n/g" | head -2 | tail -1`;
	    ## echo $PRS;
	done;
	echo -e "\e[1;32m $CMD \e[m";
	echo $CMD | /bin/sh;
    fi;
done;
