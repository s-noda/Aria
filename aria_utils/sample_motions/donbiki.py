#!/usr/bin/env python
import rospy
import time
import aria

def donbiki():
	aria.set_interpolation('slowout')
	aria.eye([0.0,0.0,1.0])
	aria.set_positions([0.0,0.25132742524147034,0.20943951606750488,0.20943951606750488,0.0,2.094395160675049,1.0,0.7539822459220886,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.20943951606750488,0.0,0.0,-0.5026548504829407,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	donbiki()

