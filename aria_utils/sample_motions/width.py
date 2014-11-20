#!/usr/bin/env python
import rospy
import time
import aria

def width():
	aria.set_interpolation('sigmoid')
	aria.eye([0.0,0.0,1.0])
	aria.set_positions([0.0,0.0,0.0,-0.04188790172338486,0.0,1.5498523712158203,0.0,0.0,0.0,0.0,0.08377580344676971,0.0,-1.5498523712158203,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	width()

