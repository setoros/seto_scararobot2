#!/usr/bin/env python
import rospy
from time import sleep
from geometry_msgs.msg import Point

def publisher():
    pub = rospy.Publisher('/beads_position', Point, queue_size=1)
    rospy.init_node('point_publisher', anonymous=True)
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():
        p = Point()

        p.x = 80
        p.y = 80
        pub.publish(p)
        sleep(3)

        p.x = -80
        p.y = -80
        pub.publish(p)
        sleep(3)

        p.x = -80
        p.y = 80
        pub.publish(p)
        sleep(3)

        p.x = 80
        p.y = -80
        pub.publish(p)
        sleep(3)

        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass






