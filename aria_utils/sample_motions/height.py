#!/usr/bin/env python
import rospy
import time
import aria

def height():
	aria.set_interpolation('linear')
	aria.eye([0.0,0.0,1.0])
	aria.set_positions([0.0,0.46076691150665283,0.0,-0.9634217619895935,0.0,1.5917402505874634,-0.6283185482025146,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	height()

