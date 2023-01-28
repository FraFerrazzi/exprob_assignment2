#!/usr/bin/env python
"""
.. module:: robot_battery_state
	:platform: Unix
	:synopsis: Python module for the management of the battery status
   
.. moduleauthor:: Francesco Ferrazzi <s5262829@studenti.unige.it>

ROS node for the first assignment of the Experimental Robotics course of the Robotics Engineering
Master program. The software architecture allows the control of the battery level of the robot.
This node publishes the state of the battery on the topic /state/battery_low. The transition of 
the battery level from high to low can happen in two different ways. 
The first one is to set battery_low = True after a random delay defined in the architecture. 
The second one is to set battery_low = True manually, retrieving the input from the user. 
When the battery becomes low, the transition is published.
The node also implements a service responsible for charging the robot after a timer expires.
The service is blocking and after the battery is charged, the response is sent to the client.

Publishes to:
	/state/battery_low the battery level of the robot
	
Service:
	/state/recharge to charge the robot
		
"""

import threading
import random
import rospy
import time

# Import constant name defined to structure the architecture.
from exprob_assignment2 import architecture_name_mapper as anm

# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_BATTERY_STATE



class RobotBatteryState:
	"""
	This class implements the robot battery state manager, which is responsible for setting the battery
	state to low and recharge the battery once the robot is in the charge location 'E'.
	
	"""
	def __init__(self):
		""" 
		Method that initializes the class RobotBatteryState.
		
		Args:
			self: instance of the current class.
		
		"""
		# Initialise this node.
		rospy.init_node(anm.NODE_ROBOT_BATTERY_STATE, log_level=rospy.INFO)
		# Initialise battery level.
		self._battery_low = False
		# Initialize the time needed to charge the battery
		self._random_battery_charge = rospy.get_param(anm.PARAM_BATTERY_CHARGE, [10.0, 15.0])
		log_msg = (f'Random-based battery charged notification: the battery will be charged (i.e., low to high) with a '
			   f'delay in the range of [{self._random_battery_charge[0]}, {self._random_battery_charge[1]}) seconds.')
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Initialize and define the service to recharge the state of the battery
		rospy.Service(anm.TOPIC_RECHARGE, SetBool, self._battery_charger)
		# Initialise randomness, if enabled.
		self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
		if self._randomness:
			self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [35.0, 50.0])
			log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
					   f'delay in the range of [{self._random_battery_time[0]}, {self._random_battery_time[1]}) seconds.')
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Start publisher on a separate thread.
		th = threading.Thread(target=self._is_battery_low)
		th.start()
		# Log information.
		log_msg = (f'Initialise node `{anm.NODE_ROBOT_BATTERY_STATE}` and topic {anm.TOPIC_BATTERY_LOW}.')
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


	def _is_battery_low(self):
		""" 
		This method publishes the changes of the battery status. This method runs on a separate 
		thread.
		
		Args:
			self: instance of the current class.
		
		"""
		# Define a `lathed` publisher to wait for initialisation and publish immediately.
		publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
		if self._randomness:
			# Publish battery level changes randomly.
			self._random_battery_notifier(publisher)
		else:
			# Publish battery level changes through a keyboard-based interface.
			self._manual_battery_notifier(publisher)
			
	
	def _battery_charger(self, request):
		""" 
		Service used to recharge the battery of the robot. Once the client gives a request to the 
		server, the server waste time to simulate the charging action for the robot's battery, which
		stays between (self._random_battery_charge[0], self._random_battery_charge[1]). After this 
		time has elapsed, the server sets the battery to high and returns a response. 
		
		Args:
			self: instance of the current class.
			request: is the boolean value used to state that the charging task should start.
		
		Returns:
			response: is the boolean value used to state that the cahrging task ended.
		
		"""
		response = SetBoolResponse()  # initialize the service response
		if request.data == True:
			log_msg = f'The battery of the robot is low... Robot RECHARGING'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			# Wait for simulate the battery charge
			delay_charge = random.uniform(self._random_battery_charge[0], self._random_battery_charge[1])
			rospy.sleep(delay_charge)
			log_msg = f'The battery of the robot was fully charged in {delay_charge} seconds'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			self._battery_low = False   
			response.success = True     
		else:
			response.success = False
		return response

	
	def _random_battery_notifier(self, publisher):
		""" 
		Publishes when the battery becomes low based on a random delay, within the interval 
		([`self._random_battery_time[0]`, `self._random_battery_time[1]`).
		The message is published through the 'publisher' input parameter which is a boolean 
		value, i.e. 'True': battery low.
		
		Args:
			self: instance of the current class.
			publisher: is the boolean value used to state the power level of the battery.
		
		"""
		while not rospy.is_shutdown():
			# If battery is full
			if self._battery_low == False:
				# Wait for simulate battery usage.
				delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
				rospy.sleep(delay)
				# Set the battery to low and publish it
				self._battery_low = True
				publisher.publish(Bool(self._battery_low))
				log_msg = f'Robot got low battery after {delay} seconds.'
				self._print_info(log_msg)
			

	def _manual_battery_notifier(self, publisher):
		""" 
		Allows a keyboard interaction with the user to emulate the change of the battery level.
		The message is published through the 'publisher' input parameter which is a boolean 
		value, i.e. 'True': battery low.
		
		Args:
			self: instance of the current class.
			publisher: is the boolean value used to state the power level of the battery.
		
		"""
		# Explain keyboard-based interaction.
		print('  # Type `Low` (`L`) to notify that the battery is low.')
		print('  # Type `cnt+C` and `Enter` to quit.')
		# Publish the default value at startup.
		publisher.publish(Bool(self._battery_low))
		# Loop to enable multiple interactions.
		while not rospy.is_shutdown():
			# Wait for the user to enter a battery state.
			user_input = input(' > ')
			user_input = user_input.lower()
			# Understand the entered text.
			error = False
			if user_input == 'low' or user_input == 'l':
				self._battery_low = True
				rospy.loginfo(anm.tag_log('Robot got low battery.', LOG_TAG))
			else:
				# Cannot understand the entered command.
				print('*** USER INPUT ERROR! Try again:')
				error = True
			# Publish the massage based on the entered command.
			if not error:
				publisher.publish(Bool(self._battery_low))

	
	def _print_info(self, msg):
		""" 
		Method which prints log informations only when the random testing is active.
		This is done to allow an intuitive usage of the keyboard-based interface.
		
		Args:
			self: instance of the current class.
			msg: is the message that will be given to the logger.
		
		"""
		if self._randomness:
			rospy.loginfo(anm.tag_log(msg, LOG_TAG))


if __name__ == "__main__":
	""" 
	Initialize the node, its service and waits a request from the client.
		
	"""  
	RobotBatteryState()
	rospy.spin()

