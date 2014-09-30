#!/usr/bin/env python

import rospy
import json
import webcommands as currentor
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class Interpolator:
    rate = 20
    goal_time = [0.0]*currentor.joint_size
    torque_joint_activity = 0 # an active joint incliments to this variable, 0 means non active
    position_joint_activity = 0
    subscribed_interpolation = [False]*currentor.joint_size # used to adjust subscribe timing
    interpolate_type_tmp = ['interpolateConstant']*currentor.joint_size # needed to avoid topic sending before goal initialization
    interpolate_type = ['interpolateConstant']*currentor.joint_size
    p = [[{'x': 0.0, 'y': 0.0}, {'x': 1.0, 'y': 1.0}]]*currentor.joint_size # interpolation function point parameters
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
    pid_initial = [{'p':0.0, 'i':0.0, 'd':0.0}]*currentor.joint_size
    pid_get = [{'p':0.0, 'i':0.0, 'd':0.0}]*currentor.joint_size
    pid_start = [{'p':0.0, 'i':0.0, 'd':0.0}]*currentor.joint_size
    pid_goal = [{'p':0.0, 'i':0.0, 'd':0.0}]*currentor.joint_size
    pid_end = [{'p':0.0, 'i':0.0, 'd':0.0}]*currentor.joint_size
    pid_delta = [{'p':0.0, 'i':0.0, 'd':0.0}]*currentor.joint_size
    counter = [10000]*currentor.joint_size
    count_finish = [0]*currentor.joint_size
    interpolate_joint = {}
    def __init__(self):
        self.interpolate_joint['interpolateConstant'] = self.interpolate_constant
        self.interpolate_joint['interpolateTLinear'] = self.interpolate_torque_linear
        self.interpolate_joint['interpolateTSlowIn'] = self.interpolate_torque_linear
        self.interpolate_joint['interpolatePLinear'] = self.interpolate_position_linear
        self.interpolate_joint['interpolatePIDLinear'] = self.interpolate_pid_linear
        id_map = rospy.get_param("/id_map")
        pid_gain = rospy.get_param("/pid_gain")
        for key in id_map:
            self.pid_initial[id_map[key]] = pid_gain[key]
            self.pid_get[id_map[key]] = pid_gain[key]
        return
    def interpolate_joints(self):
        for joint in range(0, currentor.joint_size):
            self.interpolate_joint[self.interpolate_type[joint]](joint)
        if self.torque_joint_activity > 0:
            currentor.set_torques(self.torque_goal)
        if self.position_joint_activity > 0:
            currentor.set_positions(self.position_goal)
    def set_counter(self, joint):
        self.count_finish[joint] = self.goal_time[joint] * self.rate
        if self.count_finish[joint] == 0:
            self.count_finish[joint] = 1
        self.counter[joint] += 1
    def finish_interpolation(self, joint):
        self.counter[joint] = 10000
        self.interpolate_type[joint] = 'interpolateConstant'
    def interpolate_constant(self, joint):
#        self.torque_goal[joint] = self.torque_get[joint]
#        self.position_goal[joint] = self.position_get[joint]
        return
    def interpolate_torque_linear(self, joint):
        if self.counter[joint] == 0:
            self.set_counter(joint)
            self.torque_delta[joint] = (self.torque_end[joint] - self.torque_start[joint]) / self.count_finish[joint]
            self.torque_joint_activity += 1
        elif self.counter[joint] > self.count_finish[joint]:
            self.torque_delta[joint] = 0
            self.finish_interpolation(joint)
            self.torque_joint_activity -= 1
        else:
            self.torque_goal[joint] += self.torque_delta[joint]
            self.counter[joint] += 1
    def interpolate_torque_slowin(self, joint):
        if self.counter[joint] == 0:
            self.set_counter(joint)
            self.torque_delta[joint] = self.torque_end[joint] - self.torque_start[joint]
            self.torque_joint_activity += 1
        elif self.counter[joint] > self.count_finish[joint]:
            self.torque_delta[joint] = 0
            self.finish_interpolation(joint)
            self.torque_joint_activity -= 1
        else:
            t = self.counter[joint] / self.count_finish[joint]
            tA = t / p[joint][1]['x']
            tB = (t-p[joint][1]['x']) / (1-p[joint][1]['x'])
            if t <= p[joint][1]['x']:
                self.torque_goal[joint] = (2*tA*(1-tA)*p[joint][0]['y']+tA*tA*p[joint][1]['y']) * torque_delta[joint]
            else:
                self.torque_goal[joint] = ((1-tB)*p[joint][1]['y']+tB) * torque_delta[joint]
            self.counter[joint] += 1
    def interpolate_position_linear(self, joint):
        if self.counter[joint] == 0:
            self.set_counter(joint)
            self.position_delta[joint] = (self.position_end[joint] - self.position_start[joint]) / self.count_finish[joint]
            self.position_joint_activity += 1
        elif self.counter[joint] > self.count_finish[joint]:
            self.position_delta[joint] = 0
            self.finish_interpolation(joint)
            self.position_joint_activity -= 1
        else:
            self.position_goal[joint] += self.position_delta[joint]
            self.counter[joint] += 1
    def interpolate_pid_linear(self, joint):
        if self.counter[joint] == 0:
            self.set_counter(joint)
            self.pid_delta[joint] = {'p': (self.pid_end[joint]['p'] - self.pid_start[joint]['p']) / self.count_finish[joint], 'd': (self.pid_end[joint]['d'] - self.pid_start[joint]['d']) / self.count_finish[joint], 'i': (self.pid_end[joint]['i'] - self.pid_start[joint]['i']) / self.count_finish[joint]}
        elif self.counter[joint] > self.count_finish[joint]:
            self.pid_delta[joint] = {'p':0.0, 'i':0.0, 'd':0.0}
            self.pid_get[joint] = self.pid_goal[joint]
            self.finish_interpolation(joint)
        else:
            self.pid_goal[joint] = {'p': (self.pid_goal[joint]['p'] + self.pid_delta[joint]['p']), 'i': (self.pid_goal[joint]['i'] + self.pid_delta[joint]['i']), 'd': (self.pid_goal[joint]['d'] + self.pid_delta[joint]['d'])}
            currentor.set_pid_gain(joint, self.pid_goal[joint]['p'], self.pid_goal[joint]['i'], self.pid_goal[joint]['d'])
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
    if not __interpolator__.subscribed_interpolation[0]:
        rospy.logerr('interpolator has not been defined yet')
        return
    __interpolator__.subscribed_interpolation[joint] = False
    __interpolator__.torque_start = list(__interpolator__.torque_get)
    __interpolator__.torque_goal = __interpolator__.torque_start
    __interpolator__.torque_end = list(data.data)
    __interpolator__.counter = [0]*currentor.joint_size


def goal_positions_callback(data):
    global __interpolator__
    rospy.loginfo('positions callback')
    if not __interpolator__.subscribed_interpolation[0]:
        rospy.logerr('interpolator has not been defined yet')
        return
    __interpolator__.subscribed_interpolation[joint] = False    
    __interpolator__.position_start = list(__interpolator__.position_get)
    __interpolator__.position_goal = __interpolator__.position_start
    __interpolator__.position_end = list(data.data)
    __interpolator__.counter = [0]*currentor.joint_size


def json_callback(data):
    global __interpolator__
    rospy.loginfo('json callback')
    rospy.loginfo(data.data)
    data_decode = json.loads(data.data)
    data_params = eval(data_decode['params'])
    if __interpolator__.interpolate_joint.has_key(data_decode['method']):
        if data_params[0] == 0:
            __interpolator__.goal_time = [data_params[1]]*currentor.joint_size
            __interpolator__.interpolate_type_tmp = [data_decode['method']]*currentor.joint_size
            rospy.loginfo(__interpolator__.interpolate_type[1])
            __interpolator__.subscribed_interpolation = [True]*currentor.joint_size
        else:
            joint = data_params[0]
            __interpolator__.goal_time[joint] = data_params[1]
            __interpolator__.interpolate_type_tmp[joint] = data_decode['method']
            __interpolator__.subscribed_interpolation[joint] = True
    if data_decode['method'] == 'setPosition':
        joint = data_params[0]
        if not __interpolator__.subscribed_interpolation[joint]:
            rospy.logerr('interpolator has not been defined yet')
            return
        __interpolator__.subscribed_interpolation[joint] = False
        __interpolator__.interpolate_type[joint] = __interpolator__.interpolate_type_tmp[joint]
        __interpolator__.position_start[joint] = __interpolator__.position_get[joint]
        __interpolator__.position_goal[joint] = __interpolator__.position_start[joint]
        __interpolator__.position_end[joint] = data_params[1]
        __interpolator__.counter[joint] = 0
        currentor.set_control_mode(joint, currentor.mode_pos)
    elif data_decode['method'] == 'setTorque':
        joint = data_params[0]
        if not __interpolator__.subscribed_interpolation[joint]:
            rospy.logerr('interpolator has not been defined yet')
            return
        __interpolator__.subscribed_interpolation[joint] = False
        __interpolator__.interpolate_type[joint] = __interpolator__.interpolate_type_tmp[joint]
        __interpolator__.torque_start[joint] = __interpolator__.torque_get[joint]
        __interpolator__.torque_goal[joint] = __interpolator__.torque_start[joint]
        __interpolator__.torque_end[joint] = data_params[1]
        __interpolator__.counter[joint] = 0
        currentor.set_control_mode(joint, currentor.mode_tor)
    elif data_decode['method'] == 'setPIDGain':
        joint = data_params[0]
        if not __interpolator__.subscribed_interpolation[joint]:
            rospy.logerr('interpolator has not been defined yet')
            return
        __interpolator__.subscribed_interpolation[joint] = False
        __interpolator__.interpolate_type[joint] = __interpolator__.interpolate_type_tmp[joint]
        __interpolator__.pid_start[joint] = {'p': __interpolator__.pid_get[joint]['p'], 'i': __interpolator__.pid_get[joint]['i'], 'd': __interpolator__.pid_get[joint]['d']}
        __interpolator__.pid_goal[joint] = {'p': __interpolator__.pid_start[joint]['p'], 'i': __interpolator__.pid_start[joint]['i'], 'd': __interpolator__.pid_start[joint]['d']}
        __interpolator__.pid_end[joint] = {'p': data_params[1]*__interpolator__.pid_initial[joint]['p'], 'i': data_params[2]*__interpolator__.pid_initial[joint]['i'], 'd': data_params[3]*__interpolator__.pid_initial[joint]['d']}
        __interpolator__.counter[joint] = 0
        rospy.logwarn('setting pid to %s in %f seconds' % (__interpolator__.pid_end[joint], __interpolator__.goal_time[joint]))
#        currentor.set_control_mode(joint, currentor.mode_pos)
    elif data_decode['method'] == 'interpolationParams':
        joint = data_params[0]
        rospy.loginfo(__interpolator__.subscribed_interpolation[joint])
        __interpolator__.p[joint][0] = {'x': data_params[1], 'y': data_params[2]}
        __interpolator__.p[joint][1] = {'x': data_params[3], 'y': data_params[4]}
    elif data_decode['method'] == 'setGoals':
        joint = data_params[0]
        __interpolator__.torque_goal[joint] = data_params[1]
        __interpolator__.position_goal[joint] = data_params[2]
        currentor.set_goals(joint, data_params[1], data_params[2], data_params[3])




if __name__ == '__main__':
    rospy.init_node('util_conversions', anonymous=True)
    rospy.Subscriber('/currentor_socket/sensor_array/torque', Float32MultiArray, sensor_torque_callback)
    rospy.Subscriber('/currentor_socket/sensor_array/position', Float32MultiArray, sensor_position_callback)
    rospy.Subscriber('/interpolation_wrapper/request/torque_vector', Float32MultiArray, goal_torques_callback)
    rospy.Subscriber('/interpolation_wrapper/request/position_vector', Float32MultiArray, goal_positions_callback)
    rospy.Subscriber('/interpolation_wrapper/socket_listener/json_string', String, json_callback)
    r = rospy.Rate(__interpolator__.rate)
    try:
        while not rospy.is_shutdown():
            __interpolator__.interpolate_joints()
#            rospy.loginfo(__interpolator__.torque_get)
#            rospy.loginfo(__interpolator__.position_get)
            r.sleep()
    except rospy.ROSInterruptException: pass
