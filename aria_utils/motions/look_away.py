#!/usr/bin/env python
import rospy
import time
import aria

def look_away():
	aria.set_interpolation('slowin')
	aria.set_positions([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0890854597091675,0.0,0.16755160689353943,0.25132742524147034,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	look_away()

