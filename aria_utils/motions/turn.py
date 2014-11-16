#!/usr/bin/env python
import rospy
import time
import aria

def turn():
	aria.set_interpolation('sigmoid')
	aria.set_positions([0.0,0.0,-0.6283185482025146,0.0,0.0,1.2147492170333862,-1.57,0.0,0.0,-0.7958701252937317,0.0,1.884955644607544,-1.6336281299591064,1.0,0.9634217619895935,0.53510321378707886,0.0,0.16755160689353943,0.8,0.0,0.6702064275741577,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	aria.eye([1.0,0.0,0.4])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	turn()

