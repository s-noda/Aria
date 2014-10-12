#!/bin/bash
aria_once() {
    if [ -z "$2" ] ; then
	rostopic echo -n1 "/currentor_socket/sensor_array/"$1"/data"
    else
	rostopic echo -n1 "/currentor_socket/sensor_array/"$1"/data"[$2]
    fi
}

val=$(aria_once $1 $2)
echo $val