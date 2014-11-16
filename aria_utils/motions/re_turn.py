#!/usr/bin/env python
import rospy
import time
import aria

def re_turn():
	aria.set_interpolation('sigmoid')
	aria.set_positions([0.0,0.0,0.04188790172338486,0.0,-0.5864306092262268,0.16,-1.0,0.4,0.0,0.41887903213500977,0.08377580344676971,-1.4660766124725342,-1.1728612184524536,1.2,-0.41887903213500977,-0.08377580344676971,0.0,0.0,0.0,0.0,-0.16755160689353943,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	aria.eye([0.0,0.0,0.2])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	re_turn()

