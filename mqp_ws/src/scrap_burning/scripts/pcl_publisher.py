#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
import tf.transformations


def talker():
    pub = rospy.Publisher('pcl', Path, queue_size=10)
    rospy.init_node('pcl_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    path = Path()
    start_pose_stamped = PoseStamped()
    start_pose_stamped.header.frame_id = "panda_link0"
    start_pose = Pose()
    start_pose.position.x = 0.949783265591
    start_pose.position.y = 0.035354629159
    start_pose.position.z = 0.493712007999
    # start_pose.position.x = 0.640
    # start_pose.position.y = -0.042
    # start_pose.position.z = 0.525
    # orientation = tf.transformations.quaternion_from_euler(0, 90, 0)
    # start_pose.orientation.x = orientation[0]
    # start_pose.orientation.y = orientation[1]
    # start_pose.orientation.z = orientation[2]
    # start_pose.orientation.w = orientation[3]
    # print(start_pose.orientation)
    start_pose.orientation.x = 0.653
    start_pose.orientation.y = -0.266
    start_pose.orientation.z = 0.655
    start_pose.orientation.w = -0.270
    start_pose_stamped.pose = start_pose
    path.poses.append(start_pose_stamped)
    rospy.loginfo(start_pose_stamped.header)
    # for i in range(4):
    #     new_pose = Pose()
    #     new_pose.position.x = float(0.3595 + 0.1*i)
    #     new_pose.position.y = 0
    #     new_pose.position.z = 0.643
    #     new_pose.orientation.x = 0.653
    #     new_pose.orientation.y = -0.266
    #     new_pose.orientation.z = 0.655
    #     new_pose.orientation.w = -0.270
    #     pose_stamped = PoseStamped()
    #     pose_stamped.header.frame_id = "panda_link0"
    #     pose_stamped.pose = new_pose
    #     rospy.loginfo(pose_stamped.pose)
    #     path.poses.append(pose_stamped)
    # end_pose = Pose()
    # end_pose.position.x = 0.3595
    # end_pose.position.y = 0
    # end_pose.position.z = 0.643
    # end_pose.orientation.x = 0.653
    # end_pose.orientation.y = -0.266
    # end_pose.orientation.z = 0.655
    # end_pose.orientation.w = -0.270
    # end_pose_stamped = PoseStamped()
    # end_pose_stamped.header.frame_id = "panda_link8"
    # end_pose_stamped.pose = end_pose
    # rospy.loginfo(end_pose_stamped.pose)
    # path.poses.append(end_pose_stamped)
    pub.publish(path)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
