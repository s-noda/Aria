#!/usr/bin/env python
import rospy
import time
import aria

def thinking():
	aria.set_interpolation('sigmoid')
	aria.set_positions([0.0,0.25132742524147034,0.7539822459220886,0.5445427298545837,-0.8796459436416626,2.094395160675049,-1.0,-1.626843523979187,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.2932153046131134,0.08377580344676971,0.0,0.0,0.33510321378707886,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	aria.gripper([2.0,2.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	thinking()

