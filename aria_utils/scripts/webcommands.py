#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

joint_size = 30

mode_non = 0
mode_tor = 1
mode_pos = 2

json_string_topic_name = '/ros2http/socket_listener/json_string'
mode_vector_topic_name = '/currentor_socket/request/mode_vector'
torque_vector_topic_name = '/currentor_socket/request/torque_vector'
position_vector_topic_name = '/currentor_socket/request/position_vector'

modes = [0]*joint_size
torques = [0.0]*joint_size
positions = [0.0]*joint_size

def set_control_mode(joint, mode):
    msg = String()
    msg.data = '{\"method\":\"setControlMode\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, mode)
    pub = rospy.Publisher(json_string_topic_name, String)
    pub.publish(msg)
    return mode

def set_torque(joint, torque):
    msg = String()
    msg.data = '{\"method\":\"setTorque\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, torque)
    pub = rospy.Publisher(json_string_topic_name, String)
    pub.publish(msg)
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

def set_pid_gain(joint, p, i, d):
    msg = String()
    msg.data = '{\"method\":\"setPIDGain\",\"params\":\"[%d,%f,%f,%f]\",\"id\":\"1\"}' % (joint, p, i, d)
    pub = rospy.Publisher('/ros2http/socket_listener/json_string', String)
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

if __name__ == '__main__':
    rospy.init_node('python_webcommands')
    rospy.spin()
