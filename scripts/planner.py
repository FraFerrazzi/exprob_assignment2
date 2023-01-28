#! /usr/bin/env python
"""
.. module:: planner
	:platform: Unix
	:synopsis: Python module for the planner
   
.. moduleauthor:: Francesco Ferrazzi <s5262829@studenti.unige.it>

ROS node for the first assignment of the Experimental Robotics course of the Robotics Engineering
Master program. The software architecture allows simulating the behavior of a planner that 
produces a list of random via points starting from the current position and arriving at the target 
position given by the client. Both the current position and the target positions are random points 
defined inside the environment.

Action Service:
	/motion/planner to make the planner create the desired path		
"""

import random
import rospy
# Import constant name defined to structure the architecture.
from exprob_assignment2 import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from exprob_assignment2.msg import Point, PlanFeedback, PlanResult
from exprob_assignment2.srv import GetPose
import exprob_assignment1  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER



class PlaningAction(object):
	"""
	This class implements an action server to simulate motion planning task for a surveillance robot.
	Given a current position and a target position, it generates a plan as a set of via points.
	
	"""
	def __init__(self):
		""" 
		Function that initializes the class PlaningAction.
		
		Args:
			self: instance of the current class.
		
		"""
		# Get random-based parameters used by this server
		self._random_plan_points = rospy.get_param(anm.PARAM_PLANNER_POINTS, [2, 8])
		self._random_plan_time = rospy.get_param(anm.PARAM_PLANNER_TIME, [0.1, 0.3])
		#self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
		# Instantiate and start the action server based on the `SimpleActionServer` class.
		self._as = SimpleActionServer(anm.ACTION_PLANNER, 
										exprob_assignment1.msg.PlanAction, 
										execute_cb=self.execute_callback, 
										auto_start=False)
		self._as.start()
		# Log information.
		log_msg = (f'`{anm.ACTION_PLANNER}` Action Server initialised. It will create random path with a number of point '
					f'spanning in [{self._random_plan_points[0]}, {self._random_plan_points[1]}). Each point will be generated '
					f'with a delay spanning in [{self._random_plan_time[0]}, {self._random_plan_time[1]}).')
	  
	
	def execute_callback(self, goal):
		""" 
		Callback invoked when a client sends a goal to the planner server. This method requires
		two points (i.e. the current and the target points), and returns a list of random points
		(i.e. the plan), where the fist point is the current point and the last point is the 
		target point. The plan will contain a random number of other points which spans in the range
		([`self._random_plan_points[0]`, `self._random_plan_points[1]`). To simulate computation,
		each point is added to the plan with a random delay spanning in the range 
		([`self._random_plan_time[0]`, `self._random_plan_time[1]`).
		
		Args:
			self: instance of the current class.
			goal: are the current point and the target point, which are pre defined.
		
		"""
		# Get the input parameters to compute the plan, i.e., the start (or current) and target positions.
		start_point = goal.current
		target_point = goal.target

		# Check if the start and target positions are correct. If not, this service will be aborted.
		if start_point is None or target_point is None:
			log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
			rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
			# Close service by returning an `ABORT` state to the client.
			self._as.set_aborted()
			return
		if not(self._is_valid(start_point) and self._is_valid(target_point)):
			log_msg = (f'Start point ({start_point.x}, {start_point.y}) or target point ({target_point.x}, '
								f'{target_point.y}) point out of the environment. This service will be aborted!.')
			rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
			# Close service by returning an `ABORT` state to the client.
			self._as.set_aborted()
			return
		# Initialize the "feedback" with the string point of the plan
		feedback = PlanFeedback()
		feedback.via_points = []
		feedback.via_points.append(start_point)
		# Publish the feedback and wait to simulate computation.
		self._as.publish_feedback(feedback)
		delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
		rospy.sleep(delay)

		# Get a random number of via points to be included in the plan.
		number_of_points = random.randint(self._random_plan_points[0], self._random_plan_points[1] + 1)
		log_msg = f'Server is planning {number_of_points + 1} points...'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		# Generate the points of the plan.
		for i in range(1, number_of_points):
			if self._as.is_preempt_requested():
					rospy.loginfo(anm.tag_log('Server has been cancelled by the client!', LOG_TAG))
					# Actually cancel this service.
					self._as.set_preempted()  
					return
			new_point = Point()
			new_point.x = random.uniform(0, anm.ENVIRONMENT_SIZE[0])
			new_point.y = random.uniform(0, anm.ENVIRONMENT_SIZE[1])
			feedback.via_points.append(new_point)
			if i < number_of_points - 1:
				# Publish the new random point as feedback to the client.
				self._as.publish_feedback(feedback)
				# Wait to simulate computation.
				delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
				rospy.sleep(delay)
			else:
				# Append the target point to the plan as the last point.
				feedback.via_points.append(target_point)

		# Publish the results to the client.        
		result = PlanResult()
		result.via_points = feedback.via_points
		self._as.set_succeeded(result)
		log_msg = 'Motion plan succeeded with plan: '
		log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

	
	def _is_valid(self, point):
		""" 
		Check if the point is inside the environment bounds, which are:
		x: [0, `self._environment_size[0]`], and y: [0, `self._environment_size[1]`].
		
		Args:
			self: instance of the current class.
			point: given point of coordinates (x, y) that needs to be checked
			
		Returns:
			point: returns the point's x and y coordinates if is inside the environment bounds.
		
		"""
		return 0.0 <= point.x <= anm.ENVIRONMENT_SIZE[0] and 0.0 <= point.y <= anm.ENVIRONMENT_SIZE[1]


if __name__ == '__main__':
	""" 
	Initialize the node, its action server and waits a request from the client.
		
	"""  
	rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
	server = PlaningAction()
	rospy.spin()
