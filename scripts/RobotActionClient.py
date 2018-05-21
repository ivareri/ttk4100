import threading

import rospy
import actionlib  # handles action things
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState



class RobotActionClient(object):
    """Example of controlling the floor robot using joint trajectory action."""
    def __init__(self, name, joint_names):

        # We need some info on the joint_states
        rospy.Subscriber("/" + name + "/joint_states",  # To know where we are
                         JointState,  # Message type
                         self.messageCallback)

	# Get joint names from input
	self.joint_names = joint_names;
	# Initialize list of joint_positions
        self.joint_positions = [0.0] * len(joint_names)

        # One problem with rospy subscribers is that they work as a separate
        # thread. I think this means we should ensure that joint_positions
        # are free to be edited when we want to. This means using a lock
        # from the threading library.
        self.lock = threading.Lock()

        # Now let's setup the action client
        ac = actionlib.SimpleActionClient("/" + name + "/joint_trajectory_action",
                                          FollowJointTrajectoryAction)
        self.action_client = ac

        # Wait for the server (joint controllers) to initialize
        self.action_client.wait_for_server()

    def messageCallback(self, msg):
        """That which handles joint_state messages"""
        with self.lock:
            self.joint_positions = list(msg.position)

    def goToPosition(self, joint_goal, dur, goal_tolerance=1.0):
        """Packages a joint goal into a trajectory thingy"""
        # Define the look and feel of the trajectory
        tr = JointTrajectory()
        tr.joint_names = self.joint_names
        tr.points.append(JointTrajectoryPoint())
        tr.points[0].positions = joint_goal
        tr.points[0].velocities = [0.0 for i in self.joint_names]
        tr.points[0].accelerations = [0.0 for i in self.joint_names]
        tr.points[0].time_from_start = rospy.Duration(dur)

        # Set it as a goal (separate object)
        tr_goal = FollowJointTrajectoryGoal()
        tr_goal.trajectory = tr
        tr_goal.goal_time_tolerance = rospy.Duration(goal_tolerance)
        self.action_client.send_goal(tr_goal)

    def getCurrentPosition(self):
        """Thread safe way of getting joint positions"""
        with self.lock:
            return self.joint_positions


