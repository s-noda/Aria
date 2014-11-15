#!/usr/bin/env python
import rospy
import subprocess
import re
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

joint_size = 30

mode = {'none': 0.0, 'torque': 1.0, 'position': 2.0, 'hybrid': 3.0}

feedback = {'velocity': 0.0, 'torque': 1.0, 'position': 2.0,
            'kp':3.0, 'kd': 4.0, 'ct': 5.0, 'cp':6.0}

interpolation = {'linear': 1.0, 'bezier': 2.0, 'slowout': 3.0,
                 'slowin': 4.0, 'sigmoid': 5.0, 'cubic': 6.0}

json_string_topic_name = '/ros2http/socket_listener/json_string'
mode_vector_topic_name = '/currentor_socket/request/mode_vector'
torque_vector_topic_name = '/currentor_socket/request/torque_vector'
position_vector_topic_name = '/currentor_socket/request/position_vector'
pid_vector_topic_name = '/currentor_socket/request/pid_vector'
eye_topic_name = '/2ndparty/request/eye'
gripper_topic_name = '/2ndparyt/request/gripper'


pub_json = None
pub_mode = None
pub_torque = None
pub_position = None
pub_pid = None
pub_eye = None
pub_gripper = None

def init_publisher():
    global pub_json
    global pub_mode
    global pub_torque
    global pub_position
    global pub_pid
    global pub_eye
    global pub_gripper
    pub_json = rospy.Publisher(json_string_topic_name,
                               String, latch=True)
    pub_mode = rospy.Publisher(mode_vector_topic_name,
                               Float32MultiArray, latch=True)
    pub_torque = rospy.Publisher(torque_vector_topic_name,
                                 Float32MultiArray, latch=True)
    pub_position = rospy.Publisher(position_vector_topic_name,
                                   Float32MultiArray, latch=True)
    pub_pid = rospy.Publisher(pid_vector_topic_name,
                              Float32MultiArray, latch=True)
    pub_eye = rospy.Publisher(eye_topic_name,
                              Float32MultiArray, latch=True)
    pub_gripper = rospy.Publisher(gripper_topic_name,
                                  Float32MultiArray, latch=True)

def echo_joints(data_type):
    msg = subprocess.check_output(['rostopic','echo','-n1',
                                   '/currentor_socket/sensor_array/%s/data'
                                   % (str(data_type))])
    msg = re.split('\[|\]',msg)
    msg.remove('')
    msg.remove('\n---\n')
    msg = map(float, msg[0].split(','))
    return msg

def echo_joint(data_type, joint):
    msg = subprocess.check_output(['rostopic', 'echo', '-n1', 
                                   '/currentor_socket/sensor_array/%s/data[%d]'
                                   % (str(data_type), joint)])
    msg = re.split('\[\|]|\\n',msg)
    return float(msg[0])

def set_feedback(fb_type):
    msg = String()
    msg.data = '{\"method\":\"%s\",\"params\":\"[%f]\",\"id\":\"0\"}' % ('setFeedback', fb_type)
    pub_json.publish(msg)

def set_control_mode(joint, mode):
    msg = String()
    msg.data = '{\"method\":\"setControlMode\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, mode)
    pub_json.publish(msg)
    time.sleep(0.1)
    return mode

def set_torque(joint, torque):
    msg = String()
    msg.data = '{\"method\":\"setTorque\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, torque)
    pub_json.publish(msg)
    time.sleep(0.1)
    return torque

def set_position(joint, position):
    msg = String()
    msg.data = '{\"method\":\"setPosition\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, position)
    pub_json.publish(msg)
    time.sleep(0.1)
    return position

def set_control_modes(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_mode.publish(msg)
    time.sleep(0.1)

def set_torques(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_torque.publish(msg)
    time.sleep(0.1)

def set_positions(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_position.publish(msg)
    time.sleep(0.1)

def set_pid_gain(joint, p=0.0, i=0.0, d=0.0):
    msg = Float32MultiArray()
    if not type(joint) is int:
        msg.data = joint
    else:
        set_feedback(feedback['kp'])
        time.sleep(0.1)
        tmp = echo_joints('debug')
        tmp.extend([0.0]*joint_size)
        set_feedback(feedback['kd'])
        time.sleep(0.1)
        tmp.extend(echo_joints('debug'))
        msg.data = tmp
        msg.data[joint] = p
        msg.data[joint+joint_size] = i
        msg.data[joint+2*joint_size] = d
    pub_pid.publish(msg)
    time.sleep(0.1)
    ret = echo_joints('debug')
    set_feedback(feedback['kp'])
    print echo_joints('debug')
    print ret


def init_pid_gain(joint, p, i, d):
    msg = String()
    msg.data = '{\"method\":\"initPIDGain\",\"params\":\"[%d,%f,%f,%f]\",\"id\":\"1\"}' % (joint, p, i, d)
    pub_json.publish(msg)


def set_interpolation(interpolate_type):
    msg = String()
    msg.data = '{\"method\":\"setInterpolation\",\"params\":\"[%f]\",\"id\":\"1\"}' % (interpolation[interpolate_type])
    pub_json.publish(msg)
    time.sleep(0.1)

def eye(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_eye.publish(msg)
    time.sleep(0.1)

def gripper(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_gripper.publish(msg)
    time.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('python_webcommands')
    rospy.spin()
