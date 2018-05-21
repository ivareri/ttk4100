#!/usr/bin/env python

import threading

import rospy
import actionlib  # handles action things
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from RobotActionClient import RobotActionClient

# These files contains joint name and preset joint positions
# for the robots.
import gantry
import floor

if __name__ == "__main__":
    try:

        rospy.init_node("robot_action_client", anonymous=True)

        # Start floor controller
        fac = RobotActionClient("floor", floor.joint_names)
 	gac = RobotActionClient("gantry", gantry.joint_names)
 
        # Wait until there are joint_states to listen to
        rospy.wait_for_message("/floor/joint_states", JointState, timeout=None)
	rospy.wait_for_message("/gantry/joint_states", JointState, timeout=None)

	# Let's move to home position
	fac.goToPosition(joint_goal = floor.position['home'], dur=10.0);
	gac.goToPosition(joint_goal = gantry.position['home'], dur=10.0);

        rospy.loginfo("Waiting for move to complete")
        fac.action_client.wait_for_result()
	gac.action_client.wait_for_result()

	# Let's move to pre-grab position
	fac.goToPosition(joint_goal = floor.position['pregrab'], dur=10.0);
	gac.goToPosition(joint_goal = gantry.position['pregrab'], dur=10.0);

        rospy.loginfo("Waiting for move to complete")
        fac.action_client.wait_for_result()
	gac.action_client.wait_for_result()

	# Let's grab the box
	fac.goToPosition(joint_goal = floor.position['grab'], dur=10.0);
	gac.goToPosition(joint_goal = gantry.position['grab'], dur=10.0);

        rospy.loginfo("Waiting for move to complete")
        fac.action_client.wait_for_result()
	gac.action_client.wait_for_result()

	# Let's lift the box
	fac.goToPosition(joint_goal = floor.position['lift'], dur=10.0);
	gac.goToPosition(joint_goal = gantry.position['lift'], dur=10.0);

        rospy.loginfo("Waiting for move to complete")
        fac.action_client.wait_for_result()
	gac.action_client.wait_for_result()

        # Let's rotate the box
	fac.goToPosition(joint_goal = floor.position['rotate'], dur=10.0);
	gac.goToPosition(joint_goal = gantry.position['rotate'], dur=10.0);

        rospy.loginfo("Waiting for move to complete")
        fac.action_client.wait_for_result()
	gac.action_client.wait_for_result()

	# Let's move back to home position
	fac.goToPosition(joint_goal = floor.position['home'], dur=10.0);
	gac.goToPosition(joint_goal = gantry.position['home'], dur=10.0);

        rospy.loginfo("Waiting for move to complete")
        fac.action_client.wait_for_result()
	gac.action_client.wait_for_result()

               
    except rospy.ROSInterruptException:
        # This stuff is to ensure we get the ctrl-c and treat it well
        print("Killed")
