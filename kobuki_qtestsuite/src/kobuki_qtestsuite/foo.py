import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy
import kobuki_testsuite

def bar():
    print kobuki_testsuite.bar()
    print "Foobar"
