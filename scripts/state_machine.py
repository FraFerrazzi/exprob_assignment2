#!/usr/bin/env python
"""
.. module:: state_machine
	:platform: Unix
	:synopsis: Python module for the Final State Machine
   
.. moduleauthor:: Francesco Ferrazzi <s5262829@studenti.unige.it>

ROS node for the first assignment of the Experimental Robotics course of the Robotics Engineering
Master program. The software architecture allows initializing a Final State Machine which controls 
the behavior of a surveillance robot. 
The scenario involves a robot deployed in an indoor environment for surveillance purposes.
The robot's objective is to visit different locations, which are rooms and corridors, and stay there 
for some time. The robot starts in the E, which is the charging location, and waits until it receives 
the information to build the topological map. The robot moves to a new location and waits a few seconds 
before it checks another location. This behavior is repeated until the program is not shut down.
When the robot's battery is low, it goes to the charging location and waits some time before it starts 
again the just explained behavior. When the robot's battery is not low, it should move among locations 
with the following policy:
1) It should mainly stay in corridors.
2) If a reachable room has not been visited for a fixed time, the room becomes urgent and the robot visits it.
The subscriptions, publishers, services, and service actions are defined and utilized in the helper node of
the final state machine called final_state_machine.py..
		
"""

# Import libraries
import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String, Float64, Bool, Float32

# Import the class that decouples the interface of the Finite State Machine with
# the other  nodes of the architecture from the actual implementation of the
# Finite State Machine, which is available in this file.
from exprob_assignment2.state_machine_helper import Helper

# Import constant name defined to structure the architecture.
from exprob_assignment2 import architecture_name_mapper as anm

# The list of names that identify the states of the Finite State Machine.
STATE_CHARGE = 'CHARGE'              # State where the robot recharges its battery.
STATE_BUILD_WORLD = 'BUILDWORLD'     # State where the environment is build using the ontology.
STATE_REASONER = 'REASONER'          # State that decides the next action done by the robot.
STATE_MOTION = 'MOTION'              # State that allows the robot to move in the environement according to a plan algorithm.
STATE_REACH_CHARGE = 'REACHCHARGE'   # State used to let the robot reach the charging station.
STATE_SURVEILLANCE = 'SURVEILLANCE'  # State that checks the location in which the root stops.


# The list of names that identify the transitions of the Finite State Machine.
TRANS_BATTERY_LOW = 'battery_low'      # The transition from the inner Finite State Machine associated with the `REASONER` and 'MOTION' states toward the `REACHCHARGE` state.
TRANS_BATTERY_OK = 'battery_ok'        # The transition from the `CHARGE` state to the inner Finite State Machine associated with the `REASONER` state.
TRANS_CHECK_LOC = 'check_loc'          # The transition from the `MOTION` state to the `SURVEILLANCE` state.
TRANS_INFO_DONE = 'info_done'          # The transition from the `REASONER` state to the `MOTION` state.
TRANS_WORLD_DONE = 'world_done'        # The transition from the `BUILDWORLD` state with to the 'REASONER' state.
TRANS_CHARGE_ON = 'charge_on'          # The transition from the 'REACHCHARGE' state toward the `CAHRGE` state.
TRANS_CHECK_DONE = 'check_done'        # The transition from the 'SURVEILLANCE' state toward the `REASONER` state.


# Initialize and define the tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE										  



class BuildWorld(smach.State):
	""" 
	Class that defines the state: BUILDWORLD.
		

	"""
	def __init__(self, helper):
		""" 
		Method that initializes the state BUILDWORLD.
		
		Args:
			self: instance of the current class.
			helper: instance of the class Helper() allocated in state_machine_helper.py`

		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper
								 
	def execute(self, userdata):
		""" 
		Method which is executed before exiting the state BUILDWORLD. This method generates the 
		environment by calling the method build_environment() defined in the helper node. 
		When the environment is built, the transition to the next state occurs.
		
		Args:
			self: instance of the current class.
			userdata: shared variable between the states of the Final State Machine

		Returns:
			TRANS_WORLD_DONE: is the transition to go from the BUILDWORLD state to the REASONER state.
		
		"""
		log_msg = f'\n\n############ Executing state BUILD WORLD ############\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG)) 
		self._helper.build_environment()     
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.world_done():
					return TRANS_WORLD_DONE
			finally:
					self._helper.mutex.release()
					

					
class Charge(smach.State):
	""" 
	Class that defines the state: CHARGE.
		

	"""
	def __init__(self, helper):
		""" 
		Method that initializes the state CHARGE.
		
		Args:
			self: instance of the current class.
			helper: instance of the class Helper() allocated in state_machine_helper.py`

		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper

	def execute(self, userdata):
		""" 
		Method which is executed before exiting the state CHARGE. This method makes the robot 
		charge itself relying on the method recharge_srv() defined in the helper node.
		When the battery is charged, the transition to the next state occurs.
		
		Args:
			self: instance of the current class.
			userdata: shared variable between the states of the Final State Machine

		Returns:
			TRANS_BATTERY_OK: is the transition to go from the CHARGE state to the REASONER state.
		
		"""
		rospy.loginfo('\n\n############ Executing state CHARGE ############\n')
		self._helper.recharge_srv()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if not self._helper.ret_battery_low():
					return TRANS_BATTERY_OK
			finally:
				self._helper.mutex.release()			

			

class ReachCharge(smach.State):
	""" 
	Class that defines the state: REACHCHARGE.
		

	"""
	def __init__(self, helper):
		""" 
		Method that initializes the state REACHCHARGE.
		
		Args:
			self: instance of the current class.
			helper: instance of the class Helper() allocated in state_machine_helper.py`

		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper
			
	def execute(self, userdata):
		""" 
		Method which is executed before exiting the state REACHCHARGE. This method makes the 
		robot go to the charging location 'E' by calling the method go_to_charge() defined in 
		the helper node. 
		When the robot reaches the charging location, the transition to the next state occurs.
		
		Args:
			self: instance of the current class.
			userdata: shared variable between the states of the Final State Machine

		Returns:
			TRANS_CHARGE_ON: is the transition to go from the REACHCHARGE state to the CHARGE state.
		
		"""
		rospy.loginfo('\n\n############ Executing state REACH CHARGE ############\n')
		self._helper.go_to_charge()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.charge_ready():
					return TRANS_CHARGE_ON
			finally:
				self._helper.mutex.release()
		


class Reasoner(smach.State):
	""" 
	Class that defines the state: REASONER.
		

	"""
	def __init__(self, helper):
		""" 
		Function that initializes the state REASONER.
		
		Args:
			self: instance of the current class.
			helper: instance of the class Helper() allocated in state_machine_helper.py`

		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper
			
	def execute(self, userdata):
		""" 
		Method which is executed before exiting the state REASONER. This function makes the robot
		reason in order to achieve the wanted behavior for the surveillance robot, by calling the
		method reason() defined in the helper node. When the robot finishes to query the ontology, 
		the power level of the battery is checked. If the battery is low, the next 
		state to be executed will be REACHCHARGE, else it will be executed the MOTION state.
		
		Args:
			self: instance of the current class.
			userdata: shared variable between the states of the Final State Machine

		Returns:
			TRANS_BATTERY_LOW: is the transition to go from the REASONER state to the REACHCHARGE state.
			TRANS_INFO_DONE: is the transition to go from the REASONER state to the MOTION state.
		
		"""
		log_msg = f'\n\n############ Executing state REASONER ############\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		goal_location = self._helper.reason()
		log_msg = f'The next location that will be reached is: {goal_location}\n\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.ret_battery_low():
					# may cancel goals
					return TRANS_BATTERY_LOW
				if self._helper.reason_done():
					return TRANS_INFO_DONE
			finally:
				self._helper.mutex.release()        		



class Motion(smach.State):
	""" 
	Class that defines the state: MOTION.
		

	"""
	def __init__(self, helper):
		""" 
		Function that initializes the state MOTION.
		
		Args:
			self: instance of the current class.
			helper: instance of the class Helper() allocated in state_machine_helper.py`

		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper 
			
	def execute(self, userdata):
		""" 
		Function which is executed before exiting the state MOTION. 
		
		Args:
			self: instance of the current class.
			userdata: shared variable between the states of the Final State Machine

		Returns:
			TRANS_BATTERY_LOW: is the transition to go from the MOTION state to the REACHCHARGE state.
			TRANS_CHECK_LOC: is the transition to go from the MOTION state to the SURVEILLANCE state.
		
		"""
		log_msg = f'\n\n############ Executing state MOTION ############\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		self._helper.planner()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				self._helper.check_planner()
				if self._helper.ret_battery_low():
					self._helper.planner_cli.cancel_all_goals()
					return TRANS_BATTERY_LOW
				if self._helper.plan_done():
					return TRANS_CHECK_LOC
			finally:
				self._helper.mutex.release()



class Surveillance(smach.State):
	""" 
	Class that defines the state: SURVEILLANCE.
		

	"""
	def __init__(self, helper):
		""" 
		Method that initializes the state SURVEILLANCE.
		
		Args:
			self: instance of the current class.
			helper: instance of the class Helper() allocated in state_machine_helper.py`

		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper 
		
	def execute(self, userdata):
		""" 
		Method which is executed before exiting the state SURVEILLANCE. This method simulates a
		surveillance task when the robot arrives at a specific location.
		It wastes time while it cyclically checks the state of the battery.
		If the battery is low, the next state to be executed will be REACHCHARGE, else it will be 
		executed the REASONER state.
		
		Args:
			self: instance of the current class.
			userdata: shared variable between the states of the Final State Machine

		Returns:
			TRANS_BATTERY_LOW: is the transition to go from the SURVEILLANCE state to the REACHCHARGE state.
			TRANS_CHECK_DONE: is the transition to go from the SURVEILLANCE state to the REASONER state.
		
		"""
		log_msg = f'\n\n############ Executing state SURVEILLANCE ############\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		self._helper.do_surveillance()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.ret_battery_low():
					self._helper.controller_cli.cancel_all_goals()
					return TRANS_BATTERY_LOW
				if self._helper.surveillance_done():
					return TRANS_CHECK_DONE
			finally:
				self._helper.mutex.release()


			
def main():
	"""
	This method initializes the Final State Machine of the node state_machine.py using the SMACH
	modules. Some documentation can be found online at the following link: `smach <http://wiki.ros.org/smach/>`_.
    	Every state of the node relies on the node state_machine_helper.py, in fact, an instance of the
    	Helper() situated on the node state_machine_helper.py is passed to every state of the FSM.
    	
    	"""
	rospy.init_node(anm.NODE_STATE_MACHINE, log_level=rospy.INFO)
	
	helper = Helper()
	
	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['container_interface'])
	sm.userdata.sm_counter = 0

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(STATE_BUILD_WORLD, BuildWorld(helper),
							transitions={TRANS_BATTERY_LOW:STATE_BUILD_WORLD,
								     TRANS_CHARGE_ON:STATE_BUILD_WORLD, 
								     TRANS_BATTERY_OK:STATE_BUILD_WORLD,
								     TRANS_CHECK_LOC:STATE_BUILD_WORLD,
								     TRANS_INFO_DONE:STATE_BUILD_WORLD,
								     TRANS_WORLD_DONE:STATE_REASONER,
								     TRANS_CHECK_DONE:STATE_BUILD_WORLD})
																										
		smach.StateMachine.add(STATE_CHARGE, Charge(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_CHARGE,
								     TRANS_CHARGE_ON:STATE_CHARGE,
								     TRANS_BATTERY_OK:STATE_REASONER,
								     TRANS_CHECK_LOC:STATE_CHARGE,
							             TRANS_INFO_DONE:STATE_CHARGE,
								     TRANS_WORLD_DONE:STATE_CHARGE,
								     TRANS_CHECK_DONE:STATE_CHARGE})
													
		smach.StateMachine.add(STATE_REASONER, Reasoner(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_REACH_CHARGE,
								     TRANS_CHARGE_ON:STATE_REASONER, 
								     TRANS_BATTERY_OK:STATE_REASONER,
								     TRANS_CHECK_LOC:STATE_REASONER,
								     TRANS_INFO_DONE:STATE_MOTION,
								     TRANS_WORLD_DONE:STATE_REASONER,
								     TRANS_CHECK_DONE:STATE_REASONER})
													
		smach.StateMachine.add(STATE_MOTION, Motion(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_REACH_CHARGE, 
								     TRANS_CHARGE_ON:STATE_MOTION,
							             TRANS_BATTERY_OK:STATE_MOTION,
								     TRANS_CHECK_LOC:STATE_SURVEILLANCE,
								     TRANS_INFO_DONE:STATE_MOTION,
								     TRANS_WORLD_DONE:STATE_MOTION,
								     TRANS_CHECK_DONE:STATE_MOTION})
													
		smach.StateMachine.add(STATE_REACH_CHARGE, ReachCharge(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_REACH_CHARGE, 
								     TRANS_CHARGE_ON:STATE_CHARGE,
								     TRANS_BATTERY_OK:STATE_REACH_CHARGE,
								     TRANS_CHECK_LOC:STATE_REACH_CHARGE,
								     TRANS_INFO_DONE:STATE_REACH_CHARGE,
								     TRANS_WORLD_DONE:STATE_REACH_CHARGE,
								     TRANS_CHECK_DONE:STATE_REACH_CHARGE})
										
		smach.StateMachine.add(STATE_SURVEILLANCE, Surveillance(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_REACH_CHARGE, 
								     TRANS_CHARGE_ON:STATE_SURVEILLANCE,
								     TRANS_BATTERY_OK:STATE_SURVEILLANCE,
								     TRANS_CHECK_LOC:STATE_SURVEILLANCE,
								     TRANS_INFO_DONE:STATE_SURVEILLANCE,
								     TRANS_WORLD_DONE:STATE_SURVEILLANCE,
								     TRANS_CHECK_DONE:STATE_REASONER})
										  
	# Create and start the introspection server for visualization
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()
