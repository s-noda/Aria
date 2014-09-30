#!/usr/bin/env python
import webcommands_extended as c
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
    c.webcommands.shscript = rospy.get_param("/sh_script")
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
    def set_goals(self, joint, torque, position, fb_parity=0):
        msg = String()
        msg.data = '{\"method\":\"setGoals\",\"params\":\"[%d,%f,%f,0.001]\",\"id\":\"\1\"}' % (joint, torque, position)
        pub = rospy.Publisher('/ros2http/socket_listener/json_string', String)
        pub.publish(msg)
        if fb_parity == 1:
            c.webcommands.echo_parity_fb(pub, msg, 'velocity', c.fb_pos, joint, position)
        else:
            c.webcommands.echo_parity_fb(pub, msg, 'velocity', c.fb_tor, joint, torque)
    def set_pid_gain(self, joint, p, i, d):
        c.set_pid_gain(joint, p*self.pid_gain[self.rev_id_map[joint]]['p'], i*self.pid_gain[self.rev_id_map[joint]]['i'], d*self.pid_gain[self.rev_id_map[joint]]['d'])
    """
    def table_wipe(self):
        rospy.loginfo('starting demo')
        if not self.tryq("have you initiated aria? y or n   "):
            rospy.logwarn('demo failed')
            return
        c.set_position_x(20, -math.pi/6)
        rospy.loginfo('moving torso')
        while self.tryq():
            c.set_position_x(20, -math.pi/6)
        self.set_pid_gain(20, 10.0, 0.0 ,1.0)
        time.sleep(0.1)
        self.set_pid_gain(18, 10.0, 0.0 ,1.0)
        time.sleep(0.1)
        self.set_pid_gain(19, 10.0, 0.0 ,1.0)
        time.sleep(0.1)
        c.set_position_x(2, math.pi/6)
        rospy.loginfo('moving shoulder')
        while self.tryq():
            c.set_position_x(2, math.pi/6)
        self.set_pid_gain(5, 0.0, 0.0, 1.0)
        self.tryq("press enter once right elbow reaches table")
        c.set_ct_cp(5, 1.0, 0.0)
        c.set_goals(5, -6.0, 0.0)
        c.set_control_mode(5, 3.0)
        rospy.loginfo('right elbow in torque control')
        self.set_pid_gain(7, 0.2, 0.0, 1.0)
        rospy.loginfo('right wrist lowered gain')
        c.set_position_x(1, math.pi/3)
        time.sleep(3.0)
        c.set_position_x(1, -math.pi/6)
        time.sleep(3.0)
        while not self.tryq("enter y to exit   "):
            c.set_position_x(1, math.pi/3)
            time.sleep(3.0)
            c.set_position_x(1, -math.pi/6)
            time.sleep(3.0)
        rospy.loginfo('complete')
    """
    def table_wipe_gravity(self):
        rospy.loginfo('starting demo')
        if not self.tryq("have you initiated aria? y or n   "):
            rospy.logwarn('demo failed')
            return
        c.set_position_x(20, -math.pi/6)
        rospy.loginfo('moving torso')
        while self.tryq():
            c.set_position_x(20, -math.pi/6)
        self.set_pid_gain(20, 10.0, 0.0 ,1.0)
        time.sleep(0.1)
        self.set_pid_gain(18, 10.0, 0.0 ,1.0)
        time.sleep(0.1)
        self.set_pid_gain(19, 10.0, 0.0 ,1.0)
        time.sleep(0.1)
        c.set_position_x(2, math.pi/6)
        rospy.loginfo('moving shoulder')
        while self.tryq():
            c.set_position_x(2, math.pi/6)
        self.set_pid_gain(5, 0.0, 0.0, 1.0)
        self.tryq("press enter once right elbow reaches table")
        c.set_control_mode(5, 0.0)
        #c.set_ct_cp(5, 0.0, 1.0)
        time.sleep(0.1)
        c.set_goals(5, 0.0, 0.0)
        time.sleep(0.1)
        self.set_pid_gain(5, 1.0, 0.0, 1.0)
        time.sleep(0.1)
        c.set_control_mode(5, 3.0)
        time.sleep(0.1)
        rospy.loginfo('right elbow in gravity control')
        self.set_pid_gain(7, 0.2, 0.0, 1.0)
        time.sleep(0.1)
        rospy.loginfo('right wrist lowered gain')
        c.set_position_x(1, math.pi/3)
        time.sleep(3.0)
        c.set_position_x(1, 0.0)#-math.pi/6)
        time.sleep(3.0)
        while not self.tryq("enter y to exit   "):
            c.set_position_x(1, math.pi/3)
            time.sleep(3.0)
            c.set_position_x(1, 0.0)#-math.pi/6)
            time.sleep(3.0)
        rospy.loginfo('complete')
        while self.tryq("reset pose ?   "):
            self.set_pid_gain(7, 1.0, 0.0, 1.0)
            time.sleep(0.1)
            c.set_goals(5, 0.0, math.pi/2)
            time.sleep(0.1)
            c.set_control_mode(5, 2.0)
            time.sleep(0.1)
            c.set_position_x(1, 0.0)
            time.sleep(0.1)
            c.set_position_x(2, 0.0)
            time.sleep(0.1)
            c.set_position_x(20, 0.0)
            time.sleep(0.1)
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
    c.interpolate_mode()
    #c.enable_parity()
    rospy.init_node('demo', anonymous=True)
    rospy.Subscriber('/aria/commandline', String, keyboard_callback)
    rospy.spin()
