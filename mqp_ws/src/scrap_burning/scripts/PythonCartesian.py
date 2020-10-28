#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Panda:
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        waypoints = []

        group.set_max_velocity_scaling_factor(.5)
        # group.set_planning_time(20)
        newpose = geometry_msgs.msg.Pose()
        newpose.orientation.w = 1.0
        newpose.position.x = 0.5
        newpose.position.y = 0.1
        newpose.position.z = 0.5
        waypoints.append(copy.deepcopy(newpose))
        newpose.orientation.w = 1.0
        newpose.position.x = 0.5
        newpose.position.y = 0.1
        newpose.position.z = 0.51
        waypoints.append(copy.deepcopy(newpose))
        newpose.orientation.w = 1.0
        newpose.position.x = 0.5
        newpose.position.y = 0.1
        newpose.position.z = 0.52
        waypoints.append(copy.deepcopy(newpose))
        newpose.orientation.w = 1.0
        newpose.position.x = 0.5
        newpose.position.y = 0.4
        newpose.position.z = 0.52
        waypoints.append(copy.deepcopy(newpose))
        newpose.orientation.w = 1.0
        newpose.position.x = 0.5
        newpose.position.y = 0.2
        newpose.position.z = 0.7
        waypoints.append(copy.deepcopy(newpose))
        newpose.orientation.w = 0
        newpose.position.x = 0
        newpose.position.y = 0
        newpose.position.z = 0
        waypoints.append(copy.deepcopy(newpose))

        # y = 0.5
        # z = 0.5
        # for x in range(11):
        #     print y, z
        #     if x < 5:
        #         y += 0.1
        #         z += 0.1
        #     else:
        #         y -= 0.1
        #         z -= 0.1
        #     newpose.orientation.w = 1.0
        #     newpose.position.x = 1
        #     newpose.position.y = y
        #     newpose.position.z = z
        #     waypoints.append(copy.deepcopy(newpose))
        # wpose = group.get_current_pose().pose
        # wpose.position.z -= scale * 0.1  # First move up (z)
        # waypoints.append(copy.deepcopy(wpose))
        #
        # wpose.position.y -= scale * 0.1  # and sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
        #
        # wpose.position.z += scale * 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))
        #
        # wpose.position.y += scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold


        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL


def main():
    try:
        print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
        raw_input()
        tutorial = MoveGroupPythonIntefaceTutorial()

        print "============ Press `Enter` to plan and display a Cartesian path ..."
        raw_input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        print "============ Press `Enter` to execute a saved path ..."
        raw_input()
        tutorial.execute_plan(cartesian_plan)

        print "============ Python tutorial demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
