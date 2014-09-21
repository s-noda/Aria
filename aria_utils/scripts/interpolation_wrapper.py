#!/usr/bin/env python

import rospy
import json
import webcommands as currentor
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class Interpolator:
    rate = 10
    goal_time = [0.0]*currentor.joint_size
    interpolate_type = ['interpolateConstant']*currentor.joint_size
    torque_get = [0.0]*currentor.joint_size
    torque_start = [0.0]*currentor.joint_size
    torque_goal = [0.0]*currentor.joint_size
    torque_end = [0.0]*currentor.joint_size
    torque_delta = [0.0]*currentor.joint_size
    position_get = [0.0]*currentor.joint_size
    position_start = [0.0]*currentor.joint_size
    position_goal = [0.0]*currentor.joint_size
    position_end = [0.0]*currentor.joint_size
    position_delta = [0.0]*currentor.joint_size
    counter = [0]*currentor.joint_size
    count_finish = [0]*currentor.joint_size
    set_interpolation = {}
    interpolate_joint = {}
    def __init__(self):
        self.set_interpolation['interpolateConstant'] = self.set_constant
        self.set_interpolation['interpolateTLinear'] = self.set_torque_linear
        self.set_interpolation['interpolatePLinear'] = self.set_position_linear
        self.interpolate_joint['interpolateConstant'] = self.interpolate_constant
        self.interpolate_joint['interpolateTLinear'] = self.interpolate_torque_linear
        self.interpolate_joint['interpolatePLinear'] = self.interpolate_position_linear
        return
    def interpolate_joints(self):
        for joint in range(0, currentor.joint_size):
            self.interpolate_joint[self.interpolate_type[joint]](joint)
    def set_constant(self, joint):
        return
    def set_torque_linear(self, joint):
        rospy.loginfo('set torque %d' % (joint))
        self.count_finish[joint] = self.goal_time[joint] * self.rate
        if self.count_finish[joint] == 0:
            self.count_finish[joint] = 1
        self.torque_delta[joint] = (self.torque_end[joint] - self.torque_start[joint]) / self.count_finish[joint]
    def set_position_linear(self, joint):
        self.count_finish[joint] = self.goal_time[joint] * self.rate
        if self.count_finish[joint] == 0:
            self.count_finish[joint] = 1
        self.position_delta[joint] = (self.position_end[joint] - self.position_start[joint]) / self.count_finish[joint]
    def interpolate_constant(self, joint):
        self.torque_goal[joint] = self.torque_get[joint]
        self.position_goal[joint] = self.position_get[joint]
    def interpolate_torque_linear(self, joint):
        rospy.loginfo('interpolate %d' % (joint))
        if self.counter[joint] >= self.count_finish[joint]:
            self.torque_delta[joint] = 0
            self.counter[joint] = 0
            self.interpolate_type[joint] = 'interpolateConstant'
        else:
            self.torque_goal[joint] += self.torque_delta[joint]
            self.counter[joint] += 1
    def interpolate_position_linear(self, joint):
        if self.counter[joint] >= self.count_finish[joint]:
            self.position_delta[joint] = 0
            self.counter[joint] = 0
            self.interpolate_type[joint] = 'interpolateConstant'
        else:
            self.position_goal[joint] += self.position_delta[joint]
            self.counter[joint] += 1

__interpolator__ = Interpolator()

def sensor_torque_callback(data):
    global __interpolator__
    __interpolator__.torque_get = data.data

def sensor_position_callback(data):
    global __interpolator__
    __interpolator__.position_get = data.data

def goal_torques_callback(data):
    global __interpolator__
    rospy.loginfo('torques callback')
    for joint in range(0, currentor.joint_size):
        __interpolator__.torque_start[joint] = __interpolator__.torque_get[joint]
        __interpolator__.torque_goal[joint] = __interpolator__.torque_start[joint]
        __interpolator__.torque_end[joint] = data.data[joint]
        __interpolator__.counter[joint] = 0
        rospy.loginfo(__interpolator__.interpolate_type[joint])
        __interpolator__.set_interpolation[__interpolator__.interpolate_type[joint]](joint)

def goal_positions_callback(data):
    global __interpolator__
    rospy.loginfo('positions callback')
    for joint in range(0, currentor.joint_size):
        __interpolator__.position_start[joint] = __interpolator__.position_get[joint]
        __interpolator__.position_goal[joint] = __interpolator__.position_start[joint]
        __interpolator__.position_end[joint] = data.data[joint]
        __interpolator__.counter[joint] = 0
        __interpolator__.set_interpolation[__interpolator__.interpolate_type[joint]](joint)

def json_callback(data):
    global __interpolator__
    rospy.loginfo('json callback')
    rospy.loginfo(data.data)
    data_decode = json.loads(data.data)
    data_params = eval(data_decode['params'])
    if __interpolator__.interpolate_joint.has_key(data_decode['method']):
        if data_params[0] == 0:
            __interpolator__.goal_time = [data_params[1]]*currentor.joint_size
            __interpolator__.interpolate_type = [data_decode['method']]*currentor.joint_size
            rospy.loginfo(__interpolator__.interpolate_type[1])
        else:
            joint = data_params[0]
            __interpolator__.goal_time[joint] = data_params[1]
            __interpolator__.interpolate_type[joint] = data_decode['method']
    if data_decode['method'] == 'setPosition':
        joint = data_params[0]
        __interpolator__.position_start[joint] = __interpolator__.position_get[joint]
        __interpolator__.position_goal[joint] = __interpolator__.position_start[joint]
        __interpolator__.position_end[joint] = data_params[1]
        __interpolator__.counter[joint] = 0
        __interpolator__.set_interpolation[__interpolator__.interpolate_type[joint]](joint)
        currentor.set_control_mode(joint, currentor.mode_pos)


if __name__ == '__main__':
    rospy.init_node('util_conversions', anonymous=True)
#    rospy.Subscriber('/currentor_socket/sensor_array/torque', Float32MultiArray, sensor_torque_callback)
    rospy.Subscriber('/currentor_socket/sensor_array/position', Float32MultiArray, sensor_position_callback)
    rospy.Subscriber('/interpolation_wrapper/request/torque_vector', Float32MultiArray, goal_torques_callback)
    rospy.Subscriber('/interpolation_wrapper/request/position_vector', Float32MultiArray, goal_positions_callback)
    rospy.Subscriber('/interpolation_wrapper/socket_listener/json_string', String, json_callback)
    r = rospy.Rate(__interpolator__.rate)
    __interpolator__.torque_get = [0.0, 1.0, 2.0, 1.5, 0.0, -1.0, -2.0, -1.5, 0.0, 0.0, 0.0, 1.0, -1.0, -1.0, -1.0, 2.0, 2.0, 2.0, 0.5, 0.5, -0.5, 0.7, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    try:
        while not rospy.is_shutdown():
            __interpolator__.interpolate_joints()
            currentor.set_torques(__interpolator__.torque_goal)
            currentor.set_positions(__interpolator__.position_goal)
            r.sleep()
    except rospy.ROSInterruptException: pass
