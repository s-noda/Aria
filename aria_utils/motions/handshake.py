#!/usr/bin/env python
import rospy
import time
import aria

def handshake():
	aria.set_interpolation('sigmoid')
	aria.eye([0.0,0.0,1.0])
	aria.set_positions([0.0,0.0,0.2932153046131134,0.0,0.0,1.3404128551483154,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	handshake()

