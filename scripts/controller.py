#! /usr/bin/env python
"""
.. module:: controller
	:platform: Unix
	:synopsis: Python module for the controller 
   
.. moduleauthor:: Francesco Ferrazzi <s5262829@studenti.unige.it>

ROS node for the first assignment of the Experimental Robotics course of the Robotics Engineering
Master program. The software architecture allows simulating the behavior of a controller that follows a list
of via points starting from the current point until arriving at the target point. 

Action service:
	/motion/controller to make the controller follow the desired path		
"""

import random
import rospy
# Import constant name defined to structure the architecture.
from exprob_assignment2 import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from exprob_assignment2.msg import ControlFeedback, ControlResult
from exprob_assignment2.srv import SetPose
import exprob_assignment2  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER



class ControllingAction(object):
	"""
	This class implements an action server to simulate motion controlling task for a surveillance robot.
	Given a plan as a set of via points, it simulates the movement to reach each point with a random
	delay. The server updates the current robot position when it is done.
	
	"""
	def __init__(self):
		""" 
		Function that initializes the class ControllingAction.
		
		Args:
			self: instance of the current class.
		
		"""
		# Get random-based parameters used by this server
		self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.2, 0.5])
		# Instantiate and start the action server based on the `SimpleActionServer` class.
		self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
											exprob_assignment1.msg.ControlAction,
											execute_cb=self.execute_callback,
											auto_start=False)
		self._as.start()
		# Log information.
		log_msg = (f'`{anm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a delay ' 
						f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}).')
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	
	def execute_callback(self, goal):
		""" 
		Callback invoked when a client sends a goal to the controller server. This method requires
		a list of via points (i.e. the plan), and it simulate a movement through each point with a 
		delay taht is between (['self._random_motion_time[0]`, `self._random_motion_time[1]`).
		As soon as the server is done, it sends the result to the client.
		
		Args:
			self: instance of the current class.
			goal: it is a list of via points (i.e. the plan)
		
		"""
		# Check if the provided plan is processable. If not, this service will be aborted.
		if goal is None or goal.via_points is None or len(goal.via_points) == 0:
			rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
			self._as.set_aborted()
			return

		# Construct the feedback and loop for each via point.
		feedback = ControlFeedback()
		rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
		for point in goal.via_points:
			# Check that the client did not cancel this service.
			if self._as.is_preempt_requested():
				rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
				self._as.set_preempted()
				return
			delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
			rospy.sleep(delay)
		feedback.reached_point = point
		self._as.publish_feedback(feedback)
		log_msg = f'Reaching point ({point.x}, {point.y}).'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		# Publish the results to the client.
		result = ControlResult()
		result.reached_point = feedback.reached_point
		rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
		self._as.set_succeeded(result)
		return  # Succeeded.


if __name__ == '__main__':
	""" 
	Initialize the node, its action server and waits a request from the client.
		
	"""
	rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
	server = ControllingAction()
	rospy.spin()
