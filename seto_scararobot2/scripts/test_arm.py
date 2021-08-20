# Copyright 2020-2021 SETOUCHI ROS STUDY GROUP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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






