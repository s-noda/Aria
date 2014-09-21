#!/usr/bin/env python

import rospy
import json
import webcommands as currentor
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

__rate__ = 100
__goal_time__ = 0
__interpolation_type__ = 'constant'

__interpolate__ = ['constant']*joint_size

__position_get__ = [0]*currentor.joint_size
__postiion_start__ = [0]*currentor.joint_size
__position_goal__ = [0]*currentor.joint_size
__position_end__ = [0]*currentor.joint_size
__position_delta__ = [0]*currentor.joint_size
__position_counter__ = [0]*currentor.joint_size
__position_count_finish__ = [0]*currentor.joint_size

def set_position_constant(joint):
    return

def set_position_linear(joint):
    global __position_count_finish__
    global __position_delta__
    __position_count_finish__[joint] = __goal_time__ / __rate__
    __position_delta__[joint] = (__position_end__[joint] - __position_start__[joint]) / __position_count_finish__[joint]

set_interpolation = {}
set_interpolation['constant'] = set_position_constant
set_interpolation['p_linear'] = set_position_linear


def interpolate_constant(joint):
    __position_goal__[joint] = __position_get__[joint]

def interpolatate_position_linear(joint):
    if x >= __position_count_finish__[joint]:
        global __position_delta__
        global __position_counter__ = 0
        __position_delta__[joint] = 0
        __position_counter__[joint] = 0
        __interpolate__[joint] = 'constant'
    else:
        __position_goal__[joint] += __position_delta__[joint]

interpolate = {}
interpolate['constant'] = interpolate_constant
interpolate['p_linear'] = interpolate_position_linear

def sensor_position_callback(data):
    __position_get__ = data.data

def json_callback(data):
    data_decode = json.load(data.data)
    data_params = eval(data_decode['params'])
    if data_decode['method'] == 'interpolation':
        global __goal_time__
        global __interpolation_type__
        __goal_time__ = data_params[0]
        __interpolation_type__ = data_params[1]
    if data_decode['method'] == 'setPosition':
        global __position_start__
        global __position_goal__
        global __position_end__
        global __position_counter__
        global __interpolate__
        joint = data_params[0]
        __position_start__[joint] = __position_get__[joint]
        __position_goal__[joint] = __position_start__[joint]
        __position_end__[joint] = data_params[1]
        __position_counter__ = 0
        __interpolate__[joint] = __interpolation_type__
        set_interpolation[__intepolation_type__](joint)
        



if __name__ == '__main__':
    rospy.init_node('util_conversions', anonymous=True)
    rospy.Subscriber('/currentor_socket/sensor_array/position', Float32MultiArray, sensor_position_callback)
    rospy.Subscriber('/interpolation_wrapper/request/position_vector', Float32MultiArray, goal_positions_callback)
    rospy.Subscriber('/interpolation_wrapper/socket_listener/json_string', String, json_callback)
    r = rospy.Rate(__rate__)
    while not rospy.is_shutdown():
        currentor.set_positions(__position_goal__)
        r.sleep()
