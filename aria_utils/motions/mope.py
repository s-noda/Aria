#!/usr/bin/env python
import rospy
import time
import aria

def mope():
	aria.set_interpolation('sigmoid')
	aria.set_positions([0.0,0.6283185482025146,1.8011797666549683,-0.5026548504829407,-1.7174040079116821,1.4660766124725342,-0.3769911229610443,0.0,0.0,-2.094395160675049,0.0,1.6336281299591064,-1.5498523712158203,0.0,0.41887903213500977,0.0,0.0,-0.5864306092262268,1.57,0.0,1.1728612184524536,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	mope()

