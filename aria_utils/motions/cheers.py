#!/usr/bin/env python
import rospy
import time
import aria

def cheers():
	aria.set_interpolation('sigmoid')
	aria.eye([0.0,0.0,1.0])
	aria.set_positions([0.0,0.0,0.5026548504829407,0.0,0.0,1.2566370964050293,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	aria.set_interpolation('slowin')
	time.sleep(1.0)
	aria.set_positions([0.0,0.0,1.884955644607544,0.0,0.0,0.41887903213500977,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.8])
	time.sleep(0.8)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	cheers()

