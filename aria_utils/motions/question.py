#!/usr/bin/env python
import rospy
import time
import aria

def question():
	aria.set_interpolation('bezier')
	aria.eye([0.0,0.0,1.0])
	aria.set_positions([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.16755160689353943,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	question()

