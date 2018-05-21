#!/usr/bin/env python


import threading

# rospy fungerer som et grensesnitt mellom ROS og Python.
import rospy

# grensenitt mot ROS Actions
import actionlib  # handles action things

# sensor_msgs inneholder støttefunksjoner for forskjellige sensorer
# Vi ønsker kun å bruke JointState for å finne posisjon til roboten
from sensor_msgs.msg import JointState

# TODO: Disse er mest sannsynlig unødvendig i denne fila
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Klasse som vi bruker for å kontrollere robotene.
# Denne er lagert i RobotActionClient.py
from RobotActionClient import RobotActionClient

# Disse filene inneholder navn og forhåndsdefinerte posisjoner
# som vi skal bruke i labben.
import gantry
import floor

if __name__ == "__main__":
    try:
        # For å kunne kommunisere med andre ROS programmer må vi
        # initialisere en ROS node. Den tar seg av all kommunikasjon
        # med resten av ROS
        rospy.init_node("robot_action_client", anonymous=True)

        # Her lager vi to instanser av RobotActionClient,
        # som vi bruker for bevege robotene
        fac = RobotActionClient("floor", floor.joint_names)
 	gac = RobotActionClient("gantry", gantry.joint_names)
 
        # Disse sørger for at vi venter til robot driverene er klare
        # før vi fortsetter med vår kode. /floor/joint_states inneholder
        # posisjonen til roboten. For å se innholdet kan dere kjøre
        # 'rostopic echo /floor/joint_states' i en terminal etter at 
        # dere har startet rviz og ser robotene.
        rospy.wait_for_message("/floor/joint_states", JointState, timeout=None)
	rospy.wait_for_message("/gantry/joint_states", JointState, timeout=None)


        # Resten av koden i denne fila sender en bevegelseskommando
        # til robotene, og venter til bevegelsen er utført før vi fortsetter.
        # For å gjøre labben enklere er det eneste form for synkronisering vi bruker
        # mellom robotene.

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
