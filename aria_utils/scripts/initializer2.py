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
    rev_id_map = {}
    setup_functions = {}
    def __init__(self):
        aria.init_publisher()
        self.cmdretpub = rospy.Publisher('/aria/commandline/re', String, latch=True)
        for i in range(0, len(self.id_map)):
            for key, value in self.id_map.iteritems():
                if value == i:
                    self.rev_id_map[i] = key
                    break
        self.setup_functions['servo_on'] = self.servo_on
        self.setup_functions['servo_off'] = self.servo_off
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
#        aria.set_pid_gain(self.id_map['neck1'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['neck2'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['neck3'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_r_joint1'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_r_joint2'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_r_joint3'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_r_joint5'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_r_joint7'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_l_joint1'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_l_joint2'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_l_joint3'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_l_joint5'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['arm_l_joint7'], 0.0, 0.0, 10.0)
#        time.sleep(3)
#        aria.set_pid_gain(self.id_map['body'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['hip2'], 0.0, 0.0, 10.0)
#        aria.set_pid_gain(self.id_map['hip3'], 0.0, 0.0, 10.0)
#        time.sleep(5)
#        aria.set_pid_gain(self.id_map['arm_r_joint4'], 0.0, 0.0, 0.0)
#        aria.set_pid_gain(self.id_map['arm_r_joint6'], 0.0, 0.0, 0.0)
#        aria.set_pid_gain(self.id_map['arm_l_joint4'], 0.0, 0.0, 0.0)
#        aria.set_pid_gain(self.id_map['arm_l_joint6'], 0.0, 0.0, 0.0)
#        time.sleep(3)
        aria.set_control_modes([aria.mode['torque']]*aria.joint_size)
        aria.set_torques([0.0]*aria.joint_size)
        rospy.loginfo('done servo off')
    def initiate(self):
        rospy.loginfo('start initiate')
        aria.set_control_modes([aria.mode['none']]*aria.joint_size)
        aria.set_torque(21, 0)
        aria.set_torque(22, 0)
        aria.set_position(self.id_map['neck1'], self.initial_pose['neck1'])
        aria.set_position(self.id_map['neck2'], self.initial_pose['neck2'])
        aria.set_position(self.id_map['neck3'], self.initial_pose['neck3'])
        time.sleep(1)
        aria.set_position(self.id_map['hip2'], self.initial_pose['hip2'])
        aria.set_position(self.id_map['hip3'], self.initial_pose['hip3'])
        time.sleep(1)
        aria.set_position(self.id_map['arm_r_joint1'], self.initial_pose['arm_r_joint1'])
        aria.set_position(self.id_map['arm_r_joint2'], self.initial_pose['arm_r_joint2'])
        aria.set_position(self.id_map['arm_r_joint3'], self.initial_pose['arm_r_joint3'])
        aria.set_position(self.id_map['arm_r_joint4'], self.initial_pose['arm_r_joint4'])
        aria.set_position(self.id_map['arm_r_joint5'], self.initial_pose['arm_r_joint5'])
        aria.set_position(self.id_map['arm_r_joint6'], self.initial_pose['arm_r_joint6'])
        aria.set_position(self.id_map['arm_r_joint7'], self.initial_pose['arm_r_joint7'])
        aria.set_position(self.id_map['arm_l_joint1'], self.initial_pose['arm_l_joint1'])
        aria.set_position(self.id_map['arm_l_joint2'], self.initial_pose['arm_l_joint2'])
        aria.set_position(self.id_map['arm_l_joint3'], self.initial_pose['arm_l_joint3'])
        aria.set_position(self.id_map['arm_l_joint4'], self.initial_pose['arm_l_joint4'])
        aria.set_position(self.id_map['arm_l_joint5'], self.initial_pose['arm_l_joint5'])
        aria.set_position(self.id_map['arm_l_joint6'], self.initial_pose['arm_l_joint6'])
        aria.set_position(self.id_map['arm_l_joint7'], self.initial_pose['arm_l_joint7'])
        aria.set_position(self.id_map['body'], self.initial_pose['body'])
        pos = aria.echo_joints('position')
        okay = 0
        if len(pos) < 20:
            self.cmdret('error when applying position')
            return
        for x in range(1,21):
            if not pos[x] == self.initial_pose[self.rev_id_map[x]]:
                self.cmdret('joint %d has wrong position' % (x))
            else:
                okay = okay + 1
        self.cmdret('%d' % (okay))
        rospy.loginfo('done initiate')
    def get_exist(self):
        self.servo_on()
        exist = aria.echo_joints('mode')
        okay = 0
        if len(exist) < 20:
            self.cmdret('error when checking exist')
            return
        for x in range(1,21):
            if not exist[x] == 1.0:
                self.cmdret('joint %d is missing' % (x))
            else:
                okay = okay + 1
        self.cmdret('%d' % (okay))
        rospy.loginfo('done get exist')
    def apply_gain(self):
        rospy.loginfo('start apply_gain')
        aria.init_pid_gain(self.id_map['arm_r_joint1'], self.pid_gain['arm_r_joint1']['p'], self.pid_gain['arm_r_joint1']['i'], self.pid_gain['arm_r_joint1']['d'])
        aria.init_pid_gain(self.id_map['arm_r_joint2'], self.pid_gain['arm_r_joint2']['p'], self.pid_gain['arm_r_joint2']['i'], self.pid_gain['arm_r_joint2']['d'])
        aria.init_pid_gain(self.id_map['arm_r_joint3'], self.pid_gain['arm_r_joint3']['p'], self.pid_gain['arm_r_joint3']['i'], self.pid_gain['arm_r_joint3']['d'])
        aria.init_pid_gain(self.id_map['arm_r_joint4'], self.pid_gain['arm_r_joint4']['p'], self.pid_gain['arm_r_joint4']['i'], self.pid_gain['arm_r_joint4']['d'])
        aria.init_pid_gain(self.id_map['arm_r_joint5'], self.pid_gain['arm_r_joint5']['p'], self.pid_gain['arm_r_joint5']['i'], self.pid_gain['arm_r_joint5']['d'])
        aria.init_pid_gain(self.id_map['arm_r_joint6'], self.pid_gain['arm_r_joint6']['p'], self.pid_gain['arm_r_joint6']['i'], self.pid_gain['arm_r_joint6']['d'])
        aria.init_pid_gain(self.id_map['arm_r_joint7'], self.pid_gain['arm_r_joint7']['p'], self.pid_gain['arm_r_joint7']['i'], self.pid_gain['arm_r_joint7']['d'])
        aria.init_pid_gain(self.id_map['arm_l_joint1'], self.pid_gain['arm_l_joint1']['p'], self.pid_gain['arm_l_joint1']['i'], self.pid_gain['arm_l_joint1']['d'])
        aria.init_pid_gain(self.id_map['arm_l_joint2'], self.pid_gain['arm_l_joint2']['p'], self.pid_gain['arm_l_joint2']['i'], self.pid_gain['arm_l_joint2']['d'])
        aria.init_pid_gain(self.id_map['arm_l_joint3'], self.pid_gain['arm_l_joint3']['p'], self.pid_gain['arm_l_joint3']['i'], self.pid_gain['arm_l_joint3']['d'])
        aria.init_pid_gain(self.id_map['arm_l_joint4'], self.pid_gain['arm_l_joint4']['p'], self.pid_gain['arm_l_joint4']['i'], self.pid_gain['arm_l_joint4']['d'])
        aria.init_pid_gain(self.id_map['arm_l_joint5'], self.pid_gain['arm_l_joint5']['p'], self.pid_gain['arm_l_joint5']['i'], self.pid_gain['arm_l_joint5']['d'])
        aria.init_pid_gain(self.id_map['arm_l_joint6'], self.pid_gain['arm_l_joint6']['p'], self.pid_gain['arm_l_joint6']['i'], self.pid_gain['arm_l_joint6']['d'])
        aria.init_pid_gain(self.id_map['arm_l_joint7'], self.pid_gain['arm_l_joint7']['p'], self.pid_gain['arm_l_joint7']['i'], self.pid_gain['arm_l_joint7']['d'])
        aria.init_pid_gain(self.id_map['neck1'], self.pid_gain['neck1']['p'], self.pid_gain['neck1']['i'], self.pid_gain['neck1']['d'])
        aria.init_pid_gain(self.id_map['neck2'], self.pid_gain['neck2']['p'], self.pid_gain['neck2']['i'], self.pid_gain['neck2']['d'])
        aria.init_pid_gain(self.id_map['neck3'], self.pid_gain['neck3']['p'], self.pid_gain['neck3']['i'], self.pid_gain['neck3']['d'])
        aria.init_pid_gain(self.id_map['hip2'], self.pid_gain['hip2']['p'], self.pid_gain['hip2']['i'], self.pid_gain['hip2']['d'])
        aria.init_pid_gain(self.id_map['hip3'], self.pid_gain['hip3']['p'], self.pid_gain['hip3']['i'], self.pid_gain['hip3']['d'])
        aria.init_pid_gain(self.id_map['body'], self.pid_gain['body']['p'], self.pid_gain['body']['i'], self.pid_gain['body']['d'])
        aria.init_pid_gain(self.id_map['wheel1'], self.pid_gain['wheel1']['p'], self.pid_gain['wheel1']['i'], self.pid_gain['wheel1']['d'])
        aria.init_pid_gain(self.id_map['wheel2'], self.pid_gain['wheel2']['p'], self.pid_gain['wheel2']['i'], self.pid_gain['wheel2']['d'])
        aria.set_feedback(aria.feedback['kp'])
        kp = aria.echo_joints('debug')
        okay = 0
        if len(kp) < 20:
            self.cmdret('error when applying gain')
            return
        for x in range(1,21):
            if kp[x] == 0.0:
                self.cmdret('joint %d gain not set' % (x))
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
    #__settings__.setup_functions['apply_gain']()
    rospy.spin()
