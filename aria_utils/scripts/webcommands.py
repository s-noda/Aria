#!/usr/bin/env python
import rospy
import subprocess
import re
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

joint_size = 30

mode_non = 0.0
mode_tor = 1.0
mode_pos = 2.0
mode_hyb = 3.0

fb_vel = 0.0
fb_tor = 1.0
fb_pos = 2.0
fb_kp = 3.0
fb_kd = 4.0
fb_ct = 5.0
fb_cp = 6.0

json_string_topic_name = '/ros2http/socket_listener/json_string'
mode_vector_topic_name = '/currentor_socket/request/mode_vector'
torque_vector_topic_name = '/currentor_socket/request/torque_vector'
position_vector_topic_name = '/currentor_socket/request/position_vector'
pid_vector_topic_name = '/currentor_socket/request/pid_vector'

modes = [0]*joint_size
torques = [0.0]*joint_size
positions = [0.0]*joint_size

try_till_send = False

shscript = './aria_echo.sh'

def echo_joints(data_type):
    msg = subprocess.check_output([shscript, str(data_type)])
    msg = re.split('\[|\]',msg)
    msg.remove('')
    msg.remove(' ---\n')
    msg = map(float, msg[0].split(','))
    return msg

def echo_joint(data_type, joint):
    msg = subprocess.check_output([shscript, str(data_type), str(joint)])
    msg = msg.split(' ')
    msg.remove('---\n')
    return float(msg[0])

def echo_parity(pub, msg, data_type, joint, value):
    while True:
        pub.publish(msg)
        parity = echo_joint(data_type, joint)
        if (parity == value):
            return
        time.sleep(0.1)

def set_feedback(fb_type):
    msg = String()
    msg.data = '{\"method\":\"%s\",\"params\":\"[%f]\",\"id\":\"0\"}' % ('setFeedback', fb_type)
    pub = rospy.Publisher('/ros2http/socket_listener/json_string',String)
    pub.publish(msg)

def echo_parity_fb(pub, msg, data_type, feedback_type, joint, value):
    while True:
        pub.publish(msg)
        set_feedback(feedback_type)
        parity = echo_joint(data_type, joint)
        if (parity == value):
            return
        time.sleep(0.1)

def set_control_mode(joint, mode):
    msg = String()
    msg.data = '{\"method\":\"setControlMode\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, mode)
    pub = rospy.Publisher('/ros2http/socket_listener/json_string', String)
    pub.publish(msg)
    time.sleep(0.1)
    print echo_joints('mode')
    return mode


def set_torque(joint, torque):
    msg = String()
    msg.data = '{\"method\":\"setTorque\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, torque)
    pub = rospy.Publisher(json_string_topic_name, String)
    pub.publish(msg)
    time.sleep(0.1)
    set_feedback(fb_tor)
    print echo_joints('debug')
    return torque

def set_position(joint, position):
    msg = String()
    msg.data = '{\"method\":\"setPosition\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, position)
    pub = rospy.Publisher(json_string_topic_name, String)
    pub.publish(msg)
    time.sleep(0.1)
    set_feedback(fb_pos)
    print echo_joints('debug')
    return position

def set_control_modes(data):
    modes = data
    pub = rospy.Publisher(mode_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = modes
    pub.publish(msg)
    time.sleep(0.1)
    print echo_joints('mode')

def set_torques(data):
    pub = rospy.Publisher(torque_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = data
    pub.publish(msg)
    time.sleep(0.1)
    set_feedback(fb_tor)
    print echo_joints('debug')

def set_positions(data):
    pub = rospy.Publisher(position_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = data
    pub.publish(msg)
    time.sleep(0.1)
    set_feedback(fb_pos)
    print echo_joints('debug')

def set_pid_gain(joint, p=0.0, i=0.0, d=0.0):
    pub = rospy.Publisher(pid_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    if not type(joint) is int:
        msg.data = joint
    else:
        set_feedback(fb_kp)
        time.sleep(0.1)
        tmp = echo_joints('debug')
        tmp.extend([0.0]*joint_size)
        set_feedback(fb_kd)
        time.sleep(0.1)
        tmp.extend(echo_joints('debug'))
        msg.data = tmp
        msg.data[joint] = p
        msg.data[joint+joint_size] = i
        msg.data[joint+2*joint_size] = d
    pub.publish(msg)
    time.sleep(0.1)
    ret = echo_joints('debug')
    set_feedback(fb_kp)
    print echo_joints('debug')
    print ret


def init_pid_gain(joint, p, i, d):
    msg = String()
    msg.data = '{\"method\":\"initPIDGain\",\"params\":\"[%d,%f,%f,%f]\",\"id\":\"1\"}' % (joint, p, i, d)
    pub = rospy.Publisher(json_string_topic_name, String)
    pub.publish(msg)

def set_ct(joint, ct, cp=0.0):
    pub = rospy.Publisher(ct_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    if not type(joint) is int:
        msg.data = joint
    else:
        set_feedback(fb_ct)
        time.sleep(0.1)
        tmp = echo_joints('debug')
        msg.data = tmp
        msg.data[joint] = ct
    pub.publish(msg)
    time.sleep(0.1)
    print echo_joints('debug')

if __name__ == '__main__':
    rospy.init_node('python_webcommands')
    rospy.spin()
