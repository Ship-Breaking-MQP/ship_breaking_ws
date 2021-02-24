#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path


def talker():
    pub = rospy.Publisher('pcl', Path, queue_size=10)
    rospy.init_node('pcl_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    path = Path()
    start_pose_stamped = PoseStamped()
    start_pose_stamped.header.frame_id = "panda_link8"
    start_pose = Pose()
    start_pose.position.x = 1
    start_pose.position.y = 1
    start_pose.position.z = 1
    start_pose.orientation.z = 10
    start_pose.orientation.y = 10
    start_pose.orientation.x = 10
    start_pose.orientation.w = 10
    start_pose_stamped.pose = start_pose
    path.poses.append(start_pose_stamped)
    rospy.loginfo(start_pose_stamped.header)
    # for i in range(9):
    #     new_pose = Pose()
    #     new_pose.position.x = 0.5
    #     new_pose.position.y = 0.6
    #     new_pose.position.z = (0.4 + (float(i)*0.01))
    #     new_pose.orientation.z = 0.5
    #     new_pose.orientation.y = 0.5
    #     new_pose.orientation.x = 0.5
    #     new_pose.orientation.w = 0.5
    #     pose_stamped = PoseStamped()
    #     pose_stamped.header.frame_id = "middle"
    #     pose_stamped.pose = new_pose
    #     rospy.loginfo(pose_stamped.pose)
    #     path.poses.append(pose_stamped)
    # end_pose = Pose()
    # end_pose.position.x = 0.5
    # end_pose.position.y = 0.6
    # end_pose.position.z = 0.6
    # end_pose.orientation.z = 0.5
    # end_pose.orientation.y = 0.5
    # end_pose.orientation.x = 0.5
    # end_pose.orientation.w = 0.5
    # end_pose_stamped = PoseStamped()
    # end_pose_stamped.pose = end_pose
    # rospy.loginfo(end_pose_stamped.pose)
    # path.poses.append(end_pose_stamped)
    pub.publish(path)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
