#!/usr/bin/env python
import rospy
import time
import aria

def angry():
    aria.set_interpolation('bezier')
    aria.eye([0.0,0.0,1.0])
    aria.set_positions([0.0,0.0,0.0,-0.46076691150665283,-1.6336281299591064,1.2985249757766724,-1.4660766124725342,1.5079644918441772,0.0,-0.5445427298545837,-0.04188790172338486,0.0,0.0,0.0,0.0,0.0,0.04188790172338486,0.2932153046131134,0.0,0.0,1.0053097009658813,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
    time.sleep(1.0)

if __name__ == '__main__':
    aria.init_publisher()
    rospy.init_node('aria_motion')
    angry()
