#!/usr/bin/env python
import rospy
import time
import webcommands as currentor
from std_msgs.msg import String

currentor.json_string_topic_name = '/interpolation_wrapper/socket_listener/json_string'
currentor.torque_vector_topic_name = '/interpolation_wrapper/request/torque_vector'
currentor.position_vector_topic_name = '/interpolation_wrapper/request/position_vector'

class Settings:
    id_map = rospy.get_param("/id_map")
    initial_pose = rospy.get_param("/initial_pose")
    pid_gain = rospy.get_param("/pid_gain")
    setup_functions = {}
    def __init__(self):
        self.setup_functions['send_zero'] = self.send_zero
        self.setup_functions['initiate'] = self.initiate
        self.setup_functions['apply_gain'] = self.apply_gain
        return
    def send_zero(self):
        rospy.loginfo('start send zero')
        currentor.set_interpolates('interpolateTLinear', 3.0)
        currentor.modes = [currentor.mode_tor]*currentor.joint_size
        currentor.set_control_modes(currentor.modes)
        time.sleep(0.1)
        currentor.torques = [0.0]*currentor.joint_size
        currentor.set_torques(currentor.torques)
        time.sleep(1)
        rospy.loginfo('done send zero')
    def initiate(self):
        rospy.loginfo('start initiate')
        currentor.modes = [currentor.mode_non]*currentor.joint_size
        currentor.set_control_modes(currentor.modes)
        currentor.torque = currentor.set_torque(21, 0)
        time.sleep(0.1)
        currentor.torque = currentor.set_torque(22, 0)
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['neck1'], 'interpolatePLinear', 3.1)
        currentor.set_control_mode(self.id_map['neck1'], currentor.mode_pos)
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['neck2'], 'interpolatePLinear', 3.1)
        currentor.set_control_mode(self.id_map['neck2'], currentor.mode_pos)
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['neck3'], 'interpolatePLinear', 3.1)
        currentor.set_control_mode(self.id_map['neck3'], currentor.mode_pos)
        time.sleep(0.1)
        currentor.positions[self.id_map['neck1']] = self.initial_pose['neck1']
        currentor.positions[self.id_map['neck2']] = self.initial_pose['neck2']
        currentor.positions[self.id_map['neck3']] = self.initial_pose['neck3']
        currentor.set_positions(currentor.positions)
        time.sleep(1)
        currentor.set_interpolate(self.id_map['hip2'], 'interpolatePLinear', 3.1)
        currentor.set_control_mode(self.id_map['hip2'], currentor.mode_pos)
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['hip3'], 'interpolatePLinear', 3.1)
        currentor.set_control_mode(self.id_map['hip3'], currentor.mode_pos)
        time.sleep(0.1)
        currentor.positions[self.id_map['hip2']] = self.initial_pose['hip2']
        currentor.positions[self.id_map['hip3']] = self.initial_pose['hip3']
        currentor.set_positions(currentor.positions)
        time.sleep(1)
        currentor.set_interpolate(self.id_map['arm_r_joint1'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_r_joint1']] = currentor.set_position(self.id_map['arm_r_joint1'], self.initial_pose['arm_r_joint1'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_r_joint2'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_r_joint2']] = currentor.set_position(self.id_map['arm_r_joint2'], self.initial_pose['arm_r_joint2'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_r_joint3'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_r_joint3']] = currentor.set_position(self.id_map['arm_r_joint3'], self.initial_pose['arm_r_joint3'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_r_joint4'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_r_joint4']] = currentor.set_position(self.id_map['arm_r_joint4'], self.initial_pose['arm_r_joint4'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_r_joint5'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_r_joint5']] = currentor.set_position(self.id_map['arm_r_joint5'], self.initial_pose['arm_r_joint5'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_r_joint6'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_r_joint6']] = currentor.set_position(self.id_map['arm_r_joint6'], self.initial_pose['arm_r_joint6'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_r_joint7'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_r_joint7']] = currentor.set_position(self.id_map['arm_r_joint7'], self.initial_pose['arm_r_joint7'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_l_joint1'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_l_joint1']] = currentor.set_position(self.id_map['arm_l_joint1'], self.initial_pose['arm_l_joint1'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_l_joint2'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_l_joint2']] = currentor.set_position(self.id_map['arm_l_joint2'], self.initial_pose['arm_l_joint2'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_l_joint3'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_l_joint3']] = currentor.set_position(self.id_map['arm_l_joint3'], self.initial_pose['arm_l_joint3'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_l_joint4'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_l_joint4']] = currentor.set_position(self.id_map['arm_l_joint4'], self.initial_pose['arm_l_joint4'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_l_joint5'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_l_joint5']] = currentor.set_position(self.id_map['arm_l_joint5'], self.initial_pose['arm_l_joint5'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_l_joint6'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_l_joint6']] = currentor.set_position(self.id_map['arm_l_joint6'], self.initial_pose['arm_l_joint6'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['arm_l_joint7'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['arm_l_joint7']] = currentor.set_position(self.id_map['arm_l_joint7'], self.initial_pose['arm_l_joint7'])
        time.sleep(0.1)
        currentor.set_interpolate(self.id_map['body'], 'interpolatePLinear', 3.1)
        currentor.positions[self.id_map['body']] = currentor.set_position(self.id_map['body'], self.initial_pose['body'])
        time.sleep(0.1)
        rospy.loginfo('done initiate')
    def apply_gain(self):
        rospy.loginfo('start apply_gain')
        currentor.set_pid_gain(self.id_map['arm_r_joint1'], self.pid_gain['arm_r_joint1']['p'], self.pid_gain['arm_r_joint1']['i'], self.pid_gain['arm_r_joint1']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_r_joint2'], self.pid_gain['arm_r_joint2']['p'], self.pid_gain['arm_r_joint2']['i'], self.pid_gain['arm_r_joint2']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_r_joint3'], self.pid_gain['arm_r_joint3']['p'], self.pid_gain['arm_r_joint3']['i'], self.pid_gain['arm_r_joint3']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_r_joint4'], self.pid_gain['arm_r_joint4']['p'], self.pid_gain['arm_r_joint4']['i'], self.pid_gain['arm_r_joint4']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_r_joint5'], self.pid_gain['arm_r_joint5']['p'], self.pid_gain['arm_r_joint5']['i'], self.pid_gain['arm_r_joint5']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_r_joint6'], self.pid_gain['arm_r_joint6']['p'], self.pid_gain['arm_r_joint6']['i'], self.pid_gain['arm_r_joint6']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_r_joint7'], self.pid_gain['arm_r_joint7']['p'], self.pid_gain['arm_r_joint7']['i'], self.pid_gain['arm_r_joint7']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_l_joint1'], self.pid_gain['arm_l_joint1']['p'], self.pid_gain['arm_l_joint1']['i'], self.pid_gain['arm_l_joint1']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_l_joint2'], self.pid_gain['arm_l_joint2']['p'], self.pid_gain['arm_l_joint2']['i'], self.pid_gain['arm_l_joint2']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_l_joint3'], self.pid_gain['arm_l_joint3']['p'], self.pid_gain['arm_l_joint3']['i'], self.pid_gain['arm_l_joint3']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_l_joint4'], self.pid_gain['arm_l_joint4']['p'], self.pid_gain['arm_l_joint4']['i'], self.pid_gain['arm_l_joint4']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_l_joint5'], self.pid_gain['arm_l_joint5']['p'], self.pid_gain['arm_l_joint5']['i'], self.pid_gain['arm_l_joint5']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_l_joint6'], self.pid_gain['arm_l_joint6']['p'], self.pid_gain['arm_l_joint6']['i'], self.pid_gain['arm_l_joint6']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['arm_l_joint7'], self.pid_gain['arm_l_joint7']['p'], self.pid_gain['arm_l_joint7']['i'], self.pid_gain['arm_l_joint7']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['neck1'], self.pid_gain['neck1']['p'], self.pid_gain['neck1']['i'], self.pid_gain['neck1']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['neck2'], self.pid_gain['neck2']['p'], self.pid_gain['neck2']['i'], self.pid_gain['neck2']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['neck3'], self.pid_gain['neck3']['p'], self.pid_gain['neck3']['i'], self.pid_gain['neck3']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['hip2'], self.pid_gain['hip2']['p'], self.pid_gain['hip2']['i'], self.pid_gain['hip2']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['hip3'], self.pid_gain['hip3']['p'], self.pid_gain['hip3']['i'], self.pid_gain['hip3']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['body'], self.pid_gain['body']['p'], self.pid_gain['body']['i'], self.pid_gain['body']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['wheel1'], self.pid_gain['wheel1']['p'], self.pid_gain['wheel1']['i'], self.pid_gain['wheel1']['d'])
        time.sleep(0.1)
        currentor.set_pid_gain(self.id_map['wheel2'], self.pid_gain['wheel2']['p'], self.pid_gain['wheel2']['i'], self.pid_gain['wheel2']['d'])
        time.sleep(0.1)
        rospy.loginfo('done apply_gain')

__settings__ = Settings()

def keyboard_callback(data):
    global __settings__
    if __settings__.setup_functions.has_key(data.data):
        __settings__.setup_functions[data.data]()

if __name__ == '__main__':
    rospy.init_node('configurator', anonymous=True)
    rospy.Subscriber('/aria/commandline', String, keyboard_callback)
    __settings__.setup_functions['apply_gain']()
    rospy.spin()
