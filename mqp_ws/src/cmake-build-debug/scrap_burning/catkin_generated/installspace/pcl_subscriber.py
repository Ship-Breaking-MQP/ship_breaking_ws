#!/usr/bin/env python2
import sys
import math

import moveit_commander
import rospy
from nav_msgs.msg import Path

trajectory_array = []


class PclSubscriber(object):
    trajectory_array = []

    def __init__(self):
        super(PclSubscriber, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pcl_subscriber', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def callback(self, data):
        global trajectory_array
        waypoints = []
        trajectory_waypoints = []
        move_group = self.move_group
        move_group.set_goal_tolerance(6)
        # rospy.loginfo("Received path: %s", data)
        rospy.loginfo("Received %d points", len(data.poses))
        for stamped_pose in data.poses:
            # trajectory_waypoints.append(stamped_pose.pose)
            if not self.is_close(stamped_pose.pose, waypoints):
                waypoints.append(stamped_pose.pose)
        print('There are ' + str(len(waypoints)) + ' number of waypoints')
        waypoints.sort(key=lambda waypoint: waypoint.position.y, reverse=True)

        (plan, fraction) = self.plan_cartesian_path(waypoints=waypoints)

        rospy.loginfo("Received plan")
        print plan
        # raw_input()
        self.execute_plan(plan)

    def listener(self):
        rospy.Subscriber("pcl", Path, self.callback)

        rospy.spin()

    def plan_cartesian_path(self, waypoints=None):
        if waypoints is None:
            waypoints = []
        move_group = self.move_group

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0)

        return plan, fraction

    def plan_pose_goal(self, pose=None):
        move_group = self.move_group
        move_group.set_goal_tolerance(0.1)

        print "Moving to pose " + str(pose)
        move_group.set_pose_target(pose)
        # move_group.set_goal_orientation_tolerance(6)
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)

    def is_close(self, pose, waypoints):
        tolerance = 0.0001
        if len(waypoints) < 1:
            return False
        else:
            for waypoint in waypoints:
                if abs(pose.position.x-waypoint.position.x) < tolerance \
                        and abs(pose.position.x-waypoint.position.x) < tolerance \
                        and abs(pose.position.x-waypoint.position.x) < tolerance:
                    return True
            return False


def main():
    try:
        print ""
        print "----------------------------------------------------------"
        print "Welcome to the Ship Breaking MQP"
        print "----------------------------------------------------------"
        print "Press Ctrl-D to exit at any time"
        print ""
        print "============ Press `Enter` to begin the MQP by setting up the moveit_commander ..."
        raw_input()
        pcl_subscriber = PclSubscriber()
        pcl_subscriber.listener()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
