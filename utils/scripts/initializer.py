#!/usr/bin/env python
import rospy
import time
import webcommands as currentor
from std_msgs.msg import String

__non__ = 0
__tor__ = 1
__pos__ = 2

__id_map__ = rospy.get_param("/id_map")
__initial_pose__ = rospy.get_param("/initial_pose")
__pid_gain__ = rospy.get_param("/pid_gain")


def initiate():
    currentor.modes = [__non__]*currentor.joint_size
    currentor.set_modes(currentor.modes)
    currentor.torque = currentor.set_torque(21, 0)
    time.sleep(0.1)
    currentor.torque = currentor.set_torque(22, 0)
    time.sleep(0.1)
    currentor.set_mode(__id_map__['neck1'], __pos__)
    time.sleep(0.1)
    currentor.set_mode(__id_map__['neck2'], __pos__)
    time.sleep(0.1)
    currentor.set_mode(__id_map__['neck3'], __pos__)
    time.sleep(0.1)
    currentor.positions[__id_map__['neck1']] = __initial_pose__['neck1']
    currentor.positions[__id_map__['neck2']] = __initial_pose__['neck2']
    currentor.positions[__id_map__['neck3']] = __initial_pose__['neck3']
    currentor.set_positions(currentor.positions)
    time.sleep(1)
    currentor.set_mode(__id_map__['hip2'], __pos__)
    time.sleep(0.1)
    currentor.set_mode(__id_map__['hip3'], __pos__)
    time.sleep(0.1)
    currentor.positions[__id_map__['hip2']] = __initial_pose__['hip2']
    currentor.positions[__id_map__['hip3']] = __initial_pose__['hip3']
    currentor.set_positions(currentor.positions)
    time.sleep(1)
    currentor.positions[__id_map__['arm_r_joint1']] = currentor.set_position(__id_map__['arm_r_joint1'], __initial_pose__['arm_r_joint1'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_r_joint2']] = currentor.set_position(__id_map__['arm_r_joint2'], __initial_pose__['arm_r_joint2'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_r_joint3']] = currentor.set_position(__id_map__['arm_r_joint3'], __initial_pose__['arm_r_joint3'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_r_joint4']] = currentor.set_position(__id_map__['arm_r_joint4'], __initial_pose__['arm_r_joint4'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_r_joint5']] = currentor.set_position(__id_map__['arm_r_joint5'], __initial_pose__['arm_r_joint5'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_r_joint6']] = currentor.set_position(__id_map__['arm_r_joint6'], __initial_pose__['arm_r_joint6'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_r_joint7']] = currentor.set_position(__id_map__['arm_r_joint7'], __initial_pose__['arm_r_joint7'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_l_joint1']] = currentor.set_position(__id_map__['arm_l_joint1'], __initial_pose__['arm_l_joint1'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_l_joint2']] = currentor.set_position(__id_map__['arm_l_joint2'], __initial_pose__['arm_l_joint2'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_l_joint3']] = currentor.set_position(__id_map__['arm_l_joint3'], __initial_pose__['arm_l_joint3'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_l_joint4']] = currentor.set_position(__id_map__['arm_l_joint4'], __initial_pose__['arm_l_joint4'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_l_joint5']] = currentor.set_position(__id_map__['arm_l_joint5'], __initial_pose__['arm_l_joint5'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_l_joint6']] = currentor.set_position(__id_map__['arm_l_joint6'], __initial_pose__['arm_l_joint6'])
    time.sleep(0.1)
    currentor.positions[__id_map__['arm_l_joint7']] = currentor.set_position(__id_map__['arm_l_joint7'], __initial_pose__['arm_l_joint7'])
    time.sleep(0.1)
    currentor.positions[__id_map__['body']] = currentor.set_position(__id_map__['body'], __initial_pose__['body'])
    time.sleep(0.1)

def apply_gain():
    currentor.set_pid_gain(__id_map__['arm_r_joint1'], __pid_gain__['arm_r_joint1']['p'], __pid_gain__['arm_r_joint1']['i'], __pid_gain__['arm_r_joint1']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_r_joint2'], __pid_gain__['arm_r_joint2']['p'], __pid_gain__['arm_r_joint2']['i'], __pid_gain__['arm_r_joint2']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_r_joint3'], __pid_gain__['arm_r_joint3']['p'], __pid_gain__['arm_r_joint3']['i'], __pid_gain__['arm_r_joint3']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_r_joint4'], __pid_gain__['arm_r_joint4']['p'], __pid_gain__['arm_r_joint4']['i'], __pid_gain__['arm_r_joint4']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_r_joint5'], __pid_gain__['arm_r_joint5']['p'], __pid_gain__['arm_r_joint5']['i'], __pid_gain__['arm_r_joint5']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_r_joint6'], __pid_gain__['arm_r_joint6']['p'], __pid_gain__['arm_r_joint6']['i'], __pid_gain__['arm_r_joint6']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_r_joint7'], __pid_gain__['arm_r_joint7']['p'], __pid_gain__['arm_r_joint7']['i'], __pid_gain__['arm_r_joint7']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_l_joint1'], __pid_gain__['arm_l_joint1']['p'], __pid_gain__['arm_l_joint1']['i'], __pid_gain__['arm_l_joint1']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_l_joint2'], __pid_gain__['arm_l_joint2']['p'], __pid_gain__['arm_l_joint2']['i'], __pid_gain__['arm_l_joint2']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_l_joint3'], __pid_gain__['arm_l_joint3']['p'], __pid_gain__['arm_l_joint3']['i'], __pid_gain__['arm_l_joint3']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_l_joint4'], __pid_gain__['arm_l_joint4']['p'], __pid_gain__['arm_l_joint4']['i'], __pid_gain__['arm_l_joint4']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_l_joint5'], __pid_gain__['arm_l_joint5']['p'], __pid_gain__['arm_l_joint5']['i'], __pid_gain__['arm_l_joint5']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_l_joint6'], __pid_gain__['arm_l_joint6']['p'], __pid_gain__['arm_l_joint6']['i'], __pid_gain__['arm_l_joint6']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['arm_l_joint7'], __pid_gain__['arm_l_joint7']['p'], __pid_gain__['arm_l_joint7']['i'], __pid_gain__['arm_l_joint7']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['neck1'], __pid_gain__['neck1']['p'], __pid_gain__['neck1']['i'], __pid_gain__['neck1']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['neck2'], __pid_gain__['neck2']['p'], __pid_gain__['neck2']['i'], __pid_gain__['neck2']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['neck3'], __pid_gain__['neck3']['p'], __pid_gain__['neck3']['i'], __pid_gain__['neck3']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['hip2'], __pid_gain__['hip2']['p'], __pid_gain__['hip2']['i'], __pid_gain__['hip2']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['hip3'], __pid_gain__['hip3']['p'], __pid_gain__['hip3']['i'], __pid_gain__['hip3']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['body'], __pid_gain__['body']['p'], __pid_gain__['body']['i'], __pid_gain__['body']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['wheel1'], __pid_gain__['wheel1']['p'], __pid_gain__['wheel1']['i'], __pid_gain__['wheel1']['d'])
    time.sleep(0.1)
    currentor.set_pid_gain(__id_map__['wheel2'], __pid_gain__['wheel2']['p'], __pid_gain__['wheel2']['i'], __pid_gain__['wheel2']['d'])
    time.sleep(0.1)


def keyboard_callback(data):
    if data.data == "initiate":
        initiate()

if __name__ == '__main__':
    rospy.init_node('configurator', anonymous=True)
    rospy.Subscriber('/euslisp/keyboard', String, keyboard_callback)
    apply_gain()
    rospy.spin()
