#!/usr/bin/env python
import webcommands as c
import rospy
import math
import subprocess
import time
from std_msgs.msg import String

class Demo:
    demo = {}
    brk = False
    id_map = rospy.get_param("/id_map")
    pid_gain = rospy.get_param("/pid_gain")
    c.shscript = rospy.get_param("/sh_script")
    rev_id_map = {}
    def __init__(self):
        #self.demo['demo_test'] = self.demo_test
        self.demo['table_wipe'] = self.table_wipe_gravity
        for i in range(0, len(self.id_map)):
            for key, value in self.id_map.iteritems():
                if value == i:
                    self.rev_id_map[i] = key
                    break
    def tryq(self, msg="retry? y or n   "):
        response = raw_input(msg)
        if response == 'y':
            return True
        return False
    def exitq(self):
        if self.brk:
            rospy.logwarn('interrupted and quiting')
            self.brk = False
            return True
        return False
    def set_pid_gain(joint, p=0.0, i=0.0, d=0.0):
        c.set_pid_gain(joint, p, i, d)
        time.sleep(0.1)
    def set_position(joint, goal, t=3.0):
        c.set_time(t)
        time.sleep(0.1)
        c.set_position(goal)
        time.sleep(t)
    def set_torque(joint, goal, t=3.0):
        c.set_time(t)
        time.sleep(0.1)
        c.set_torque(goal)
        time.sleep(t)
    def set_control_mode(joint, mode):
        c.set_control_mode(joint, mode)
        time.sleep(0.1)
    def table_wipe_gravity(self):
        rospy.loginfo('starting demo')
        if not self.tryq("have you initiated aria? y or n   "):
            rospy.logwarn('demo failed')
            return
        self.set_position(20, -math.pi/6)
        rospy.loginfo('moving torso')
        while self.tryq():
            self.set_position(20, -math.pi/6)
        self.set_pid_gain(20, 10.0, 0.0 ,1.0)
        self.set_pid_gain(18, 10.0, 0.0 ,1.0)
        self.set_pid_gain(19, 10.0, 0.0 ,1.0)
        self.set_position(2, math.pi/6)
        rospy.loginfo('moving shoulder')
        while self.tryq():
            self.set_position(2, math.pi/6)
        self.set_pid_gain(5, 0.0, 0.0, 10.0)
        self.tryq("press enter once right elbow reaches table")
        self.set_control_mode(5, 0.0)
        self.set_position(5, 0.0, 0.1)
        self.set_pid_gain(5, 1.0, 0.0, 1.0)
        self.set_control_mode(5, 3.0)
        rospy.loginfo('right elbow in gravity control')
        self.set_pid_gain(7, 0.2, 0.0, 1.0)
        rospy.loginfo('right wrist lowered gain')
        self.set_position(1, math.pi/3)
        self.set_position(1, 0.0)#-math.pi/6)
        while not self.tryq("enter y to exit   "):
            self.set_position(1, math.pi/3)
            self.set_position(1, 0.0)#-math.pi/6)
        rospy.loginfo('complete')
        while self.tryq("reset pose ?   "):
            self.set_pid_gain(7, 1.0, 0.0, 1.0)
            self.set_position(5, math.pi/2, 0.1)
            self.set_position(1, 0.0, 0.1)
            self.set_position(2, 0.0, 0.1)
            self.set_position(20, 0.0, 0.1)
        rospy.loginfo('ready for next demo')
    def door_block(self):
        rospy.loginfo('ready for next demo')     


__demo__ = Demo()

def keyboard_callback(data):
    global __demo__
    if __demo__.demo.has_key(data.data):
        __demo__.demo[data.data]()
    elif data.data == 'break':
        __demo__.brk = True

if __name__ == '__main__':
    rospy.init_node('demo', anonymous=True)
    rospy.Subscriber('/aria/commandline', String, keyboard_callback)
    rospy.spin()
