#!/usr/bin/env sh

NODE_NAMES=[];
NODE_NAMES[0]="aria_commander";
NODE_NAMES[1]="aria_pendulum_state";
NODE_NAMES[2]="aria_viewer";
NODE_NAMES[3]="ros_invented_pendulum_node";

echo "[invented_pendulum node check]";
for name in ${NODE_NAMES[@]};
do
    if [ "`rosnode list | grep $name`" ];
    then
	echo -e -n "\t\e[1;32malive\e[m";
    else
	echo -e -n "\t\e[1;31mdead\e[m";
    fi;
    echo -e "\t -- $name";
done;
