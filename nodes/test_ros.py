#!/usr/bin/env python3
# license removed for brevity

import rospy
from std_msgs.msg import String

class Talker():
    def __init__(self):
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
    
    def run(self):
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.pub.publish(hello_str)
            self.rate.sleep()    

if __name__ == '__main__':
    try:
        talker = Talker()
        talker.run()
    except rospy.ROSInterruptException:
        pass