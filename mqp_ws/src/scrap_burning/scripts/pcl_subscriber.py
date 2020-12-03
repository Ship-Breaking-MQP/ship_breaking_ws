#!/usr/bin/env python
import sys

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

        group_name = "arm"
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
        print ""

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def callback(self, data):
        global trajectory_array
        rospy.loginfo("Received path: %s", data)
        rospy.loginfo("Received %d points", len(data.poses))
        for i in data.poses:
            trajectory_array.append(i.pose)
        (plan, fraction) = self.plan_cartesian_path(waypoints=trajectory_array)

        rospy.loginfo("Received plan")
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

    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)


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
