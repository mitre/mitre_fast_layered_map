#!/usr/bin/env python

"""
Test script that has been used to help test the receipt of marker messages
for various maps.

This code may be removed at any time as it is not and should never be necessary.
"""



import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('/object_detection', Marker, queue_size=10)
    pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(3) # 3hz
    i = 0
    while not rospy.is_shutdown():
        i+=1
        marker = Marker()
        marker.header.frame_id = "utm_enu"
        marker.header.seq = i
        marker.header.stamp = rospy.Time.now()
        marker.type = 2 # Sphere
        marker.action = 0 # Add
        marker.color.r = 100
        marker.lifetime =  rospy.Duration(0) # Forever
        marker.frame_locked = True

        marker.scale.x = 5
        marker.scale.y = 20
        marker.scale.z = 20

        marker.pose.position.x = 265465
        marker.pose.position.y = 4334002
        marker.pose.position.z = -115


        # Goal pose for planning
        pose = PoseStamped()
        pose.header.frame_id = "utm_enu"
        pose.header.seq = i
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 265469
        pose.pose.position.y = 4333989
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1

        # After 10 seconds add obstacle
        if (i > 30):
            rospy.loginfo("Publishing marker")
            pub.publish(marker)
        else:
            rospy.loginfo("Waiting to publish marker")

        pose_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass