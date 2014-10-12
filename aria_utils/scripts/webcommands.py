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
pid_string_topic_name = '/ros2http/socket_listener/json_string'
mode_vector_topic_name = '/currentor_socket/request/mode_vector'
torque_vector_topic_name = '/currentor_socket/request/torque_vector'
position_vector_topic_name = '/currentor_socket/request/position_vector'

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

def enable_parity():
    global try_till_send
    try_till_send = True

def disable_parity():
    global try_till_send
    try_till_send = False

def set_control_mode(joint, mode):
    msg = String()
    msg.data = '{\"method\":\"setControlMode\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, mode)
    pub = rospy.Publisher('/ros2http/socket_listener/json_string', String)
    if try_till_send:
        echo_parity(pub, msg, 'voltage', joint, mode)
    else:
        pub.publish(msg)
    return mode

def set_goals(joint, torque, position, rez=0.001):
    msg = String()
    msg.data = '{\"method\":\"setGoals\",\"params\":\"[%d,%f,%f,%f]\",\"id\":\"1\"}' % (joint, torque, position, rez)
    #pub = rospy.Publisher('/ros2http/socket_listener/json_string', String)
    pub = rospy.Publisher(json_string_topic_name, String)
    pub.publish(msg)

def set_torque(joint, torque):
    msg = String()
    msg.data = '{\"method\":\"setTorque\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, torque)
    pub = rospy.Publisher(json_string_topic_name, String)
    return torque

def set_position(joint, position):
    msg = String()
    msg.data = '{\"method\":\"setPosition\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, position)
    pub = rospy.Publisher(json_string_topic_name, String)
    pub.publish(msg)
    return position

def set_control_modes(data):
    modes = data
    pub = rospy.Publisher(mode_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = modes
    pub.publish(msg)

def set_torques(data):
    pub = rospy.Publisher(torque_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = data
    pub.publish(msg)

def set_positions(data):
    pub = rospy.Publisher(position_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = data
    pub.publish(msg)

def set_pid_gain(joint, p, i=0.0, d=0.0):
    pub = rospy.Publisher(pid_vector_topic_name, Float32MultiArray)
    msg = Float32MultiArray()
    if not type(joint) is int:
        msg.data = p
    else:
        set_feedback(fb_kp)
        tmp = echo_joints('debug')
        tmp.extend([0.0]*joint_size)
        set_feedback(fb_kd)
        tmp.extend(echo_joints('debug'))
        msg.data = tmp
        msg.data[joint] = p
        msg.data[joint+joint_size] = i
        msg.data[joint+2*joint_size] = d
    pub.publish(msg)

    
def init_pid_gain(joint, p, i, d):
    msg = String()
    msg.data = '{\"method\":\"initPIDGain\",\"params\":\"[%d,%f,%f,%f]\",\"id\":\"1\"}' % (joint, p, i, d)
    pub = rospy.Publisher(pid_string_topic_name, String)
    pub.publish(msg)

def set_ct_cp(joint, ct, cp, fb_parity=0):
    set_control_mode(joint, 0.0)
    msg = String()
    msg.data = '{\"method\":\"%s\",\"params\":\"[%d, %f, %f]\",\"id\":\"0\"}' % ('setCtCp', joint, ct, cp)
    pub = rospy.Publisher('/ros2http/socket_listener/json_string',String)
    if try_till_send:
        if fb_parity == 1:
            echo_parity_fb(pub, msg, 'velocity', fb_cp, joint, cp)
        else:
            echo_parity_fb(pub, msg, 'velocity', fb_ct, joint, ct)
    else:
        pub.publish(msg)

def set_interpolate(joint, interpolation_type, time):
    msg = String()
    msg.data = '{\"method\":\"%s\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (interpolation_type, joint, time)
    pub = rospy.Publisher('/interpolation_wrapper/socket_listener/json_string', String)
    pub.publish(msg)

def set_interpolates(interpolation_type, time):
    msg = String()
    msg.data = '{\"method\":\"%s\",\"params\":\"[0,%f]\",\"id\":\"0\"}' % (interpolation_type, time)
    pub = rospy.Publisher('/interpolation_wrapper/socket_listener/json_string', String)
    pub.publish(msg)

def set_interpolate_params(joint, x1, y1, x2, y2):
    msg = String()
    msg.data = '{\"method\":\"%s\",\"params\":\"[%d, %f, %f, %f, %f]\",\"id\":\"0\"}' % ('interpolationParams', joint, x1, y1, x2, y2)
    pub = rospy.Publisher('/interpolation_wrapper/socket_listener/json_string', String)
    pub.publish(msg)

def set_params(joint, arg1):
    msg = String()
    msg.data = '{\"method\":\"setParams\",\"params\":\"[%d,%f]\",\"id\":\"\1\"}' % (joint, arg1)
    pub = rospy.Publisher('/ros2http/socket_listener/json_string', String)
    pub.publish(msg)

def original_mode():
    global json_string_topic_name
    global pid_string_topic_name
    global mode_vector_topic_name
    global torque_vector_topic_name
    global position_vector_topic_name
    json_string_topic_name = '/ros2http/socket_listener/json_string'
    pid_string_topic_name = '/ros2http/socket_listener/json_string'
    mode_vector_topic_name = '/currentor_socket/request/mode_vector'
    torque_vector_topic_name = '/currentor_socket/request/torque_vector'
    position_vector_topic_name = '/currentor_socket/request/position_vector'

def interpolate_mode():
    global json_string_topic_name
    global pid_string_topic_name
    global mode_vector_topic_name
    global torque_vector_topic_name
    global position_vector_topic_name
    json_string_topic_name = '/interpolation_wrapper/socket_listener/json_string'
    pid_string_topic_name = '/interpolation_wrapper/socket_listener/json_string'
    torque_vector_topic_name = '/interpolation_wrapper/request/torque_vector'
    position_vector_topic_name = '/interpolation_wrapper/request/position_vector'


if __name__ == '__main__':
    rospy.init_node('python_webcommands')
    rospy.spin()
