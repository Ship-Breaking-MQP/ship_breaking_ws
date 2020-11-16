#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose


def talker():
    pub = rospy.Publisher('pcl', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    start_pose_stamped = PoseStamped()
    start_pose_stamped.header.frame_id = "start"
    start_pose = Pose()
    start_pose.position.x = 0.5
    start_pose.position.y = 0.5
    start_pose.position.z = 0.5
    start_pose.orientation.z = 0.5
    start_pose.orientation.y = 0.5
    start_pose.orientation.x = 0.5
    start_pose.orientation.w = 0.5
    start_pose_stamped.pose = start_pose
    pub.publish(start_pose_stamped)
    rate.sleep()
    for i in range(9):
        new_pose = Pose()
        new_pose.position.x = 0.5
        new_pose.position.y = 0.6
        new_pose.position.z = (0.4 + (float(i)*0.01))
        new_pose.orientation.z = 0.5
        new_pose.orientation.y = 0.5
        new_pose.orientation.x = 0.5
        new_pose.orientation.w = 0.5
        pose_stamped = PoseStamped()
        pose_stamped.pose = new_pose
        pose_stamped.header.frame_id = "not end"
        rospy.loginfo(pose_stamped.pose)
        pub.publish(pose_stamped)
        rate.sleep()
    end_pose = Pose()
    end_pose.position.x = 0.5
    end_pose.position.y = 0.6
    end_pose.position.z = 0.6
    end_pose.orientation.z = 0.5
    end_pose.orientation.y = 0.5
    end_pose.orientation.x = 0.5
    end_pose.orientation.w = 0.5
    pose_stamped = PoseStamped()
    pose_stamped.pose = end_pose
    pose_stamped.header.frame_id = "end"
    rospy.loginfo(pose_stamped.pose)
    pub.publish(pose_stamped)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
