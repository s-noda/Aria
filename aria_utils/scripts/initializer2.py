#!/usr/bin/env python
import rospy
import time
import aria
from std_msgs.msg import String

class Settings:
    id_map = rospy.get_param("/id_map")
    initial_pose = rospy.get_param("/initial_pose")
    pid_gain = rospy.get_param("/pid_gain")
    cmdretpub = 0.0
    setup_functions = {}
    def __init__(self):
        aria.init_publisher()
        self.cmdretpub = rospy.Publisher('/aria/commandline/re', String, latch=True)
        self.setup_functions['servo_on'] = self.servo_on
        self.setup_functions['servo_off'] = self.servo_off
        self.setup_functions['get_size'] = self.get_size
        self.setup_functions['initiate'] = self.initiate
        self.setup_functions['get_exist'] = self.get_exist
        self.setup_functions['apply_gain'] = self.apply_gain
        return
    def cmdret(self, remsg):
        msg = String()
        msg.data = remsg
        self.cmdretpub.publish(msg)
    def servo_on(self):
        msg = String()
        msg.data = '{\"method\":\"sendZero\",\"params\":\"0\",\"id\":\"1\"}'
        pub = rospy.Publisher('/ros2http/socket_listener/json_string', String, latch=True)
        pub.publish(msg)
    def servo_off(self):
        rospy.loginfo('start servo off')
#        for key, value in self.id_map.iteritems():
#            if 'wheel' in key:
#                aria.set_torque(value, 0)
#            else:
#                aria.set_pid_gain(value, 0.0, 0.0, 10.0)        
        aria.set_control_modes([aria.mode['torque']]*aria.joint_size)
        aria.set_torques([0.0]*aria.joint_size)
        rospy.loginfo('done servo off')
    def initiate(self):
        rospy.loginfo('start initiate')
        aria.set_control_modes([aria.mode['none']]*aria.joint_size)
        for key, value in self.id_map.iteritems():
            if 'wheel' in key:
                aria.set_torque(value, 0)
            else:
                aria.set_position(value, self.initial_pose[key])
        rospy.loginfo('done initiate')
    def get_size(self):
        self.cmdret('%d' % (len(self.id_map)))
        rospy.loginfo('return size')
    def get_exist(self):
        self.servo_on()
        exist = aria.echo_joints('mode')
        okay = 0
        if len(exist) < len(self.id_map):
            self.cmdret('error when applying position')
            return
        for key, value in self.id_map.iteritems():
            if not exist[value] == 1.0:
                self.cmdret('joint %d is missing' % (value))
            else:
                okay = okay + 1
        self.cmdret('%d' % (okay))
        rospy.loginfo('done get exist')
    def apply_gain(self):
        rospy.loginfo('start apply_gain')
        for key, value in self.id_map.iteritems():
                aria.init_pid_gain(value, self.pid_gain[key]['p'], self.pid_gain[key]['i'], self.pid_gain[key]['d'])
        aria.set_feedback(aria.feedback['kp'])
        kp = aria.echo_joints('debug')
        okay = 0
        if len(kp) < len(self.id_map):
            self.cmdret('error when applying position')
            return
        for key, value in self.id_map.iteritems():
            if kp[value] == 0.0:
                self.cmdret('joint %d gain not set' % (value))
            else:
                okay = okay + 1
        self.cmdret('%d' % (okay))
        rospy.loginfo('done apply_gain')

__settings__ = Settings()

def keyboard_callback(data):
    global __settings__
    if __settings__.setup_functions.has_key(data.data):
        __settings__.setup_functions[data.data]()

if __name__ == '__main__':
    rospy.init_node('configurator', anonymous=True)
    rospy.Subscriber('/aria/commandline', String, keyboard_callback)
    #__settings__.setup_functions['servo_on']()
    rospy.spin()
