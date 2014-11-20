#!/usr/bin/env python
import rospy
import time
import aria

def guruguru():
	aria.set_interpolation('sigmoid')
	aria.eye([0.8,0.8,0.3])
	aria.set_positions([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.12566371262073517,0.0,0.0,0.08377580344676971,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(0.3)
	aria.eye([-0.8,0.6,0.3])
	time.sleep(0.3)
	aria.eye([-0.6,-0.8,0.3])
	time.sleep(0.3)
	aria.eye([0.8,-0.6,0.3])
	time.sleep(0.3)
	aria.eye([0.6,0.8,0.3])
	aria.set_positions([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.12566371262073517,0.0,0.0,-0.08377580344676971,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(0.3)
	aria.eye([-0.8,0.6,0.3])
	time.sleep(0.3)
	aria.eye([-0.6,-0.8,0.3])
	time.sleep(0.3)
	aria.eye([0.8,-0.6,0.3])
	time.sleep(0.3)
	aria.eye([0.0,0.0,0.3])
	aria.set_positions([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	guruguru()

