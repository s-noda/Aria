#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

joint_size = 30
modes = [0]*joint_size
torques = [0]*joint_size
positions = [0]*joint_size

def set_mode(joint, mode):
    msg = String()
    msg.data = '{\"method\":\"setControlMode\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, mode)
    pub = rospy.Publisher('ros2http/socket_listener/json_string', String)
    pub.publish(msg)
    return mode

def set_torque(joint, torque):
    msg = String()
    msg.data = '{\"method\":\"setTorque\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, torque)
    pub = rospy.Publisher('ros2http/socket_listener/json_string', String)
    pub.publish(msg)
    return torque

def set_position(joint, position):
    msg = String()
    msg.data = '{\"method\":\"setPosition\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, position)
    pub = rospy.Publisher('ros2http/socket_listener/json_string', String)
    pub.publish(msg)
    return position

def set_modes(data):
    modes = data
    pub = rospy.Publisher('/currentor_socket/request/mode_vector', Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = modes
    pub.publish(msg)

def set_torques(data):
    pub = rospy.Publisher('/currentor_socket/request/torque_vector', Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = data
    pub.publish(msg)

def set_positions(data):
    pub = rospy.Publisher('/currentor_socket/request/position_vector', Float32MultiArray)
    msg = Float32MultiArray()
    msg.data = data
    pub.publish(msg)

def set_pid_gain(joint ,p, i, d):
    msg = String()
    msg.data = '{\"method\":\"setPIDGain\",\"params\":\"[%d,%f,%f,%f]\",\"id\":\"1\"}' % (joint, p, i, d)
    pub = rospy.Publisher('ros2http/socket_listener/json_string', String)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('python_webcommands')
    rospy.spin()
