#!/usr/bin/env python
import rospy
import time
import aria

def okay():
	aria.set_interpolation('sigmoid')
	aria.eye([0.0,0.0,1.0])
	aria.set_positions([0.0,0.5864306092262268,0.0,-2.294395160675049,1.5498523712158203,1.4660766124725342,1.0,0.5,-0.5864306092262268,0.0,2.294395160675049,-1.5498523712158203,-1.4660766124725342,-1.0,0.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	okay()

