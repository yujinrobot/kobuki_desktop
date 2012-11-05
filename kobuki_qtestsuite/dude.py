#!/usr/bin/env python
import roslib; 
roslib.load_manifest('kobuki_qtestsuite')
#from kobuki_testsuite import TravelForward
from kobuki_qtestsuite.foo import bar
#import kobuki_testsuite
#from kobuki_qtestsuite.climbing_frame import ClimbingFrame
import rospy

#def foo():
#    print "Foo"

if __name__ == '__main__':
    print "Dude"
    bar()
    print "Dudette"
