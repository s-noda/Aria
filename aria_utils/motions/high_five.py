#!/usr/bin/env python
import rospy
import time
import aria

def high_five():
	aria.set_interpolation('sigmoid')
	aria.eye([0.0,0.0,1.0])
	aria.set_positions([0.0,0.12566371262073517,2.294395160675049,0.0,0.0,0.5,-1.5498523712158203,-0.5471975803375244,0.0,0.5445427298545837,-0.12566371262073517,0.20943951606750488,-2.094395160675049,-0.7958701252937317,0.8796459436416626,0.0,-0.08377580344676971,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	high_five()

