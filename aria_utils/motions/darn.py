#!/usr/bin/env python
import rospy
import time
import aria

def darn():
	aria.set_interpolation('bezier')
	aria.set_positions([0.0,0.7120943069458008,1.2566370964050293,-0.7120943069458008,-0.12566371262073517,2.094395160675049,1.4660766124725342,0.15132742524147034,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.33510321378707886,0.0,0.0,-0.25132742524147034,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	darn()

