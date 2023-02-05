#!/usr/bin/env python
"""
.. module:: state_machine_helper
	:platform: Unix
	:synopsis: Python module for the Helper of the State Machine
   
.. moduleauthor:: Francesco Ferrazzi <s5262829@studenti.unige.it>

ROS node for the first assignment of the Experimental Robotics course of the Robotics Engineering
Master program. The software architecture allows initializing a helper class for the Final State Machine 
which controls the behavior of a surveillance robot. 
This node allows to have cleaner and more readable code in the state_machine.py node, in fact, every task
called in the previously mentioned code is defined in the current node.

Subscribes to:
	/state/battery_low where the state of the battery is published
	
Service:
	/state/recharge to charge the robot
	/armor_interface_srv to communicate with the ontology
	
Action Service:
	/motion/planner to make the planner create the desired path
	/motion/controller to make the controller follow the desired path
"""

import threading
import random
import rospy
import rospkg
import os
import time
import sys
import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from threading import Lock
from actionlib import SimpleActionClient

# Import the message to rotate the camera during the surveillance phase
from sensor_msgs.msg import JointState

# Import constant name defined to structure the architecture.
from exprob_assignment2 import architecture_name_mapper as anm

# Import the messages used by services and publishers.
from std_msgs.msg import Bool, Float64, Float32
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

# Import services and messages to build the world
from exprob_assignment2.srv import WorldInit, WorldInitResponse
from exprob_assignment2.msg import RoomConnection

# Armor import to work with the ontology
from armor_msgs.srv import ArmorDirective, ArmorDirectiveRequest, ArmorDirectiveResponse
from armor_msgs.srv import ArmorDirectiveList, ArmorDirectiveListRequest, ArmorDirectiveListResponse

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE

# get the file path for rospy_tutorials
rp = rospkg.RosPack()
assignment_path = rp.get_path('exprob_assignment2')

# Define the file path in which the ontology is stored
ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "topological_map.owl")
ONTOLOGY_FILE_PATH_DEBUG = os.path.join(assignment_path, "topological_map", "topological_map_debug.owl")
WEB_PATH = 'http://bnc/exp-rob-lab/2022-23'

# Initialize and define the client to use armor
cli_armorontology = rospy.ServiceProxy('/armor_interface_srv', ArmorDirective) 
# Initialize and define the request message for armor
armorontology_req = ArmorDirectiveRequest()
# Initialize and define the arg list to pass to the ontology
ARGS = []
# Define the number for which the state of the action client is done
DONE = 3 # since the get_state() function returns 3 when the action server achieves the goal



def ontology_manager(command, primary_command_spec, secondary_command_spec, ARGS):
	""" 
	Function used to communicate with the ARMOR service to set and retrieve informations of the ontology
	regarding the environment. This function is used instead of the ARMOR API.
		
	Args:
		command: it is the command to execute (e.g. ADD, LOAD, ...).
		primary_command_spec: it is the primary command specification (optional).
		secondary_command_spec: it is the secondary command specification (optional).
		ARGS: it is the list of arguments (e.g. list of individuals to add).

	Returns:
		armorontology_res: it returns a list of queried objects.
		
	"""
	armorontology_req.armor_request.client_name = 'example'
	armorontology_req.armor_request.reference_name = 'ontoRef'
	armorontology_req.armor_request.command = command
	armorontology_req.armor_request.primary_command_spec = primary_command_spec
	armorontology_req.armor_request.secondary_command_spec = secondary_command_spec
	armorontology_req.armor_request.args = ARGS
	rospy.wait_for_service('/armor_interface_srv')
	try:
			armorontology_res = (cli_armorontology(armorontology_req)).armor_response.queried_objects
			return armorontology_res
	except rospy.ServiceException as e:
			print('Service call failed: %s' %e)
			sys.exit(1)
			
 	
def ontology_format(old_list, start, end):
	""" 
	Function that takes as input a list and returns a new one, which starts from the old one.
	The new list takes every element of the old list, starting from the index specified by 'start'
	and finishing at the index specified by 'end'. In this way, only characters and numbers in 
	indexes that are between 'start' and 'end' will be copied into the new list.
		
	Args:
		old_list: is the list that needs to be formatted.
		start: starting list's element index that will be copied to the new list.
		end: ending list's element index that will be copied to the new list.

	Returns:
		new_list: is the correctly formatted list.
		
	"""
	new_list = [num[start:end] for num in old_list]
	return new_list



class Helper:
	"""
	This class is created to decouple the implementation of the Finite State Machine, allowing to have a
	more readable and cleaner code in the state_machine.py node. This class manages the synchronization 
	with subscribers, services and action servers to achieve the correct behavior.
	
	"""
	def __init__(self):
		""" 
		Function that initializes the class Helper.
		
		Args:
			self: instance of the current class.
		
		"""
		# Initialize the variables used in the class 
		self.battery_low = False            # Set to True if the battery of the robot is low
		self.map_completed = False          # Set to True when the ontology is complete
		self.aruco_detected = False         # Set to true when all the aruco markers have been detected
		self.reasoner_done = False          # Set to True after querying the ontology
		self.motion_completed = False       # Set to True when the robot arrives in the desired location
		self.charge_reached = False         # Set to True when the charging station is reached
		self.check_completed = False        # Set to True when the robot has finished surveillance action
		
		self._rooms_coord = []                 # List to store the coordinates X and Y of each room 
		self._connections = []                 # List of connections containing _connected_to and _through_door lists
		self._connected_to = []                # List of rooms connected to a specific room
		self._through_door = []                # List of doors used for connecting rooms
		self._rooms = []                       # List of all rooms 
		self._doors = []                       # List of all doors 
		self._next_goal = ''                   # Variable to store the name of the room that will be visited
		self._prev_goal = anm.INIT_LOCATION    # Previous location, the robot starts from location 'E'
		self.markers_detected = 0              # Count the number of detected markers
		self.charge_loc = anm.CHARGE_LOCATION  # Define the charging location
		
		self.rate = rospy.Rate(10) # 10hz
		
		# Initialize the current time
		self.timer_now = str(int(time.time()))  
		# Initialize and define the mutex to work with transition variables
		self.mutex = Lock()
		
		# Load the ontology
		ARGS = [ONTOLOGY_FILE_PATH, WEB_PATH, 'true', 'PELLET', 'false']
		ontology_manager('LOAD', 'FILE', '', ARGS)	
		log_msg = f'Loading of the ontology went well'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		
		# Subscribe to the topic that controls the battery level.
		self.battery_sub = rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.battery_callback)
		
		# Publisher to move rotate the camera for surveillance action
		self.pub_base_joint = rospy.Publisher(anm.TOPIC_JOINT_BASE, Float64, queue_size = 10)
		
		# Publisher to make the robot explore the initial room
		self.twist_pub = rospy.Publisher(anm.TOPIC_TWIST, Twist, queue_size = 10)
		
		# Initialize and define the server to build the map
		self.world_srv = rospy.Service(anm.TOPIC_WORLD, WorldInit, self.handle_world_init)
		
		# Initialize and define the client for the recharge service
		rospy.wait_for_service(anm.TOPIC_RECHARGE)
		self.recharge_cli = rospy.ServiceProxy(anm.TOPIC_RECHARGE, SetBool)
		
		# Initialize and define the action client for the move base action service
		self.move_cli = actionlib.SimpleActionClient(anm.ACTION_MOTION, MoveBaseAction)
		self.move_cli.wait_for_server()
			
		
		
	def handle_world_init(self, request):
		""" 
		Service used to 
		
		Args:
			self: instance of the current class.
			request: 
		
		Returns:
			response: 
		
		"""
		self._rooms.append(request.room)
		self._rooms_coord.append([request.x, request.y])
		for i in range(len(request.connections)):
			self._connected_to = request.connections[i].connected_to
			self._through_door = request.connections[i].through_door
			new_element = [self._connected_to, self._through_door]
			if new_element not in self._connections:
				self._connections.append(new_element)
			ARGS = ['hasDoor', self._connections[i][0], self._connections[i][1]]
			ontology_manager('ADD', 'OBJECTPROP', 'IND', ARGS)
		self.markers_detected = self.markers_detected + 1
		if self.markers_detected == anm.MARKERS_NUMBER:
			# Debug
			log_msg = f'############################'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			log_msg = f'ROOM: {self._rooms}'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			log_msg = f'COORDINATES: {self._rooms_coord}'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			log_msg = f'CONNECTIONS: {self._connections}'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			log_msg = f'############################'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			self.aruco_detected = True     
		return WorldInitResponse(status = True)
	
		
	def aruco_done(self):
		""" 
		Get the value of the variable responsible for stating the creation of the environment 
		using the ARMOR service to define the ontology.
		The returning value will be `True` if the map was created, `False` otherwise.
		
		Args:
			self: instance of the current class.
		
		Returns:
			self.map_completed: Bool value that states the status of the generation of the map.
		
		"""
		return self.aruco_detected
			
		
	def build_environment(self):
		""" 
		Method that initializes the environment ontology using the ARMOR service.
		It creates the desired indoor environment in a random way, based on a fixed number 
		of rooms, doors and corridors. 
		It also communicates with the ontology to initialize and define everything 
		that will be needed to guarantee the correct behavior of the program.
		
		Args:
			self: instance of the current class.
		
		"""
		# Get the number of rooms
		rooms_number = range(0,len(self._rooms))
		# Disjoint rooms and doors
		ARGS = self._rooms + self._doors + ['Robot1']
		ontology_manager('DISJOINT', 'IND', '', ARGS)
		# State the robot initial position
		ARGS = ['isIn', 'Robot1', self.charge_loc]
		ontology_manager('ADD', 'OBJECTPROP', 'IND' , ARGS)
		# Get a time in the past (before the timestamp of the robot)
		self.timer_now = str(int(1000000000)) # This is done to make every room URGENT at the beginning  
		# Start the timestamp in every location to retrieve when a location becomes urgent
		for i in rooms_number:
			ARGS = ['visitedAt', self._rooms[i], 'Long', self.timer_now]
			ontology_manager('ADD', 'DATAPROP', 'IND', ARGS)
			# Reason about the ontology to assign the timestamp
			ARGS = ['']
			ontology_manager('REASON', '', '', ARGS)
			ARGS = ['visitedAt', self._rooms[i]]
			last_location = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
			last_location = ontology_format(last_location, 1, 11) 
			print('Room:', self._rooms[i], '<#>  Init timestamp:', last_location, '\n')
		# Update the timestamp of corridor 'E' since the robot spawns in it
		#ARGS = ['']
		#ontology_manager('REASON', '', '', ARGS)
		#ARGS = ['visitedAt', self.charge_loc]
		#last_location = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
		#last_location = ontology_format(last_location, 1, 11) 
		self.timer_now = str(int(time.time())) # initial location is not urgent
		ARGS = ['visitedAt', self.charge_loc, 'Long', self.timer_now, last_location[0]]
		ontology_manager('REPLACE', 'DATAPROP', 'IND', ARGS)
		# Save ontology for DEBUG purposes
		#ARGS = [ONTOLOGY_FILE_PATH_DEBUG] # <--- uncomment this line for ontology debug
		#ontology_manager('SAVE', '', '', ARGS) # <--- uncomment this line for ontology debug
		log_msg = f'The map has been generated in the ontology\n\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Before continuing make the robot rotate on itself to understand the sorroundings
		self.explore_init_room() 
		self.map_completed = True   # Set to True only the one involved in the state
		
		
	def explore_init_room(self):
		cmd = Twist()
		# No linear motion
		cmd.linear.x = 0
		cmd.linear.y = 0
		cmd.linear.z = 0
		# Rotate on itself
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = -1.6
		# start the timer
		start_time = rospy.get_time()
		log_msg = f'Rotate around a bit to explore!\n\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Run the loop for six seconds to allow the ROBOT to undertand its sorroundings
		while rospy.get_time() - start_time < 9:
			self.twist_pub.publish(cmd)
			self.rate.sleep()
		# Stop rotation
		cmd.angular.z = 0
		self.twist_pub.publish(cmd)
		
		
	def world_done(self):
		""" 
		Get the value of the variable responsible for stating the creation of the environment 
		using the ARMOR service to define the ontology.
		The returning value will be `True` if the map was created, `False` otherwise.
		
		Args:
			self: instance of the current class.
		
		Returns:
			self.map_completed: Bool value that states the status of the generation of the map.
		
		"""
		return self.map_completed
		
			
	def reason(self):
		""" 
		Method that communicates with the ontology already created to retrieve information
		and decide, based on the desired pre-determined behavior, where the robot should
		move next.
		First of all, reachable rooms and their status (e.g. ROOM, URGENT, CORRIDOR) are retrieved.
		Then, each reachable room is checked and the robot will move first in URGENT locations.
		If there are no URGENT locations, it stays on CORRIDORS. If there are no CORRIDORS the robot
		moves to a random ROOM. In the end, the next location that will be visited is returned.
		
		Args:
			self: instance of the current class.
			
		Returns:
			self.next_loc: is the next location that will be reached decided by the reasoner.
		
		"""
		# Reset the boolean variables
		self.reset_var()
		# Reason about the onoloy
		ARGS = ['']
		ontology_manager('REASON', '', '', ARGS)
		# Retreive the locations that the robot can reach
		ARGS = ['canReach', 'Robot1']
		can_reach = ontology_manager('QUERY', 'OBJECTPROP', 'IND', ARGS)
		can_reach = ontology_format(can_reach, 32, -1)
		random.shuffle(can_reach) # Make the choice randomic
		log_msg = f'The Robot can reach:\n {can_reach}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Retrieve the status of the reachable locations
		loc_status = []
		all_status = []
		for loc in range(0, len(can_reach)):
			ARGS = [can_reach[loc], 'false']
			loc_status = ontology_manager('QUERY', 'CLASS', 'IND', ARGS)  
			loc_status = ontology_format(loc_status, 32, -1)
			all_status.append(loc_status)
		log_msg = f'Status of the locations:\n {all_status}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Check the status of the room (e.g. ROOM, CORRIDOR, URGENT)
		urgent_loc = []
		possible_corridor = []
		for sta in range(0, len(all_status)):
			for urg in range(0, len(all_status[sta])):
				# If location is urgent and it is reachable
				if all_status[sta][urg] == 'URGENT':
					urgent_loc.append(can_reach[sta])
				# If location is a corridor and it is reachable
				elif all_status[sta][urg] == 'CORRIDOR':
					possible_corridor.append(can_reach[sta])
		# Retrieve the next location taht will be checked by the robot
		if len(urgent_loc) == 0:
			log_msg = f'There are no urgent locations'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			if len(possible_corridor) == 0:
				log_msg = f'There are no reachable corridors'
				rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
				self._next_goal = can_reach # take the first randomic reachable room
			else:
				log_msg = f'The reachable corridors are:\n {possible_corridor}'
				rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
				self._next_goal = possible_corridor # take the first reachable corridor
		else:
			log_msg = f'The Urgent locations are:\n {urgent_loc}'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			self._next_goal = urgent_loc # take the first randomic urgent room
		if type(self._next_goal) == list:
			self._next_goal = self._next_goal[0]
		self.reasoner_done = True   # Set to True only the one involved in the state
		return self._next_goal
		
		
	def reason_done(self):
		""" 
		Get the value of the variable responsible for stating the completion of the reasoning
		phase achieved using the ARMOR service to retrieve informations from the ontology.
		The returning value will be `True` if the reasoner is done, `False` otherwise.
		
		Args:
			self: instance of the current class.
		
		Returns:
			self.reasoner_done: Bool value that states the status of the reasoner.
		
		"""
		return self.reasoner_done
	
	
	def go_to_charge(self):
		""" 
		Function that allows the robot to go to the charging location before it starts the 
		charging routine.
		When the robot's battery is low, it gets as target location the charging station
		and moves towards it. After calling the planner and the controller to reach the location,
		once 'E' is reached, the variable charge_reached is set to True and the robot is ready
		to be charged. 
		
		Args:
			self: instance of the current class.
			
		
		"""
		# Reset the boolean variables
		self.reset_var()
		# Set the next location to be the charging station
		self._next_goal = self.charge_loc
		log_msg = f'Battery of the robot low!\nThe ROBOT is going to the charging station'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		self.go_to_goal()
		while self.move_cli.get_state() != DONE: # Loops until the plan action service is Not DONE
			self.rate.sleep() # Wate time
		log_msg = f'The ROBOT arrived at the charging station'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		self.charge_reached = True   # Set to True only the one involved in the state
		
		
	def charge_ready(self):
		""" 
		Get the value of the variable responsible for stating that the robot is ready to be
		charged once the location 'E' is reached.
		The returning value will be `True` if the charge location is reached, `False` otherwise.
		
		Args:
			self: instance of the current class.
		
		Returns:
			self.charge_reached: Bool value that states if the charging location is reached.
		
		"""
		return self.charge_reached
		
	
	def battery_callback(self, msg):
		""" 
		It is the callback that manages the subscriber to the topic: /state/battery_low to retrieve
		the state of the battery.
		
		Args:
			self: instance of the current class.
			msg: is the subscriber to the topic /state/battery_low to get the state of the battery.
		
		"""
		self.mutex.acquire()    # take the mutex
		try: 
			self.battery_low = msg.data    # change the flag of battery low with the received message
			if self.battery_low == True:
				log_msg = f'\n@@@ Battery of the robot is low! Recharging needed @@@\n'
				rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			if self.battery_low == False:
				log_msg = f'\n@@@ Battery of the robot is full! @@@\n'
				rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		finally:
			self.mutex.release()    # release the mutex
	
			
	def ret_battery_low(self):
		""" 
		Get the value of the variable responsible for stating the power level of the battery
		of the robot. 
		The returning value will be `True` if the battery is low, `False` otherwise.
		
		Args:
			self: instance of the current class.
		
		Returns:
			self.battery_low: Bool value that states the status of the battery.
		
		"""
		return self.battery_low
			
			
	def recharge_srv(self):
		""" 
		Blocking service used to charge the battery of the robot. Once the battery is low 
		and the robot is in the charging location, a request is sent to the service which 
		charges the battery after a defined time and gets a result as soon as it is charged. 
		When the service is done, the battery of the robot is set to high by putting the variable
		battery_low to False.
		
		Args:
			self: instance of the current class.
		
		"""
		request = SetBoolRequest()
		request.data = True
		response = self.recharge_cli(request)
		log_msg = f'The Robot has been recharged! Ready for action!!\n\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		self.battery_low = False
	
		
	def go_to_goal(self):
		""" 
		This method executes a planner for a surveillance task. It starts by deciding a random
		point inside the environment that will be reached. Then, a request to the PlanGoal() action 
		service is done to simulate a planner.
		
		Args:
			self: instance of the current class.
		
		"""
		# Reset the boolean variables
		self.reset_var()
		# Get the goal and its coordinates
		try:
			room_index = self._rooms.index(self._next_goal)
		except:
			log_msg = f'The Goal cannot be found\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		log_msg = f'The ROBOT is planning to go to:\nRoom: {self._rooms[room_index]}\nCoordinates: ({self._rooms_coord[room_index][0]},{self._rooms_coord[room_index][1]})\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		goal = MoveBaseGoal()
		# Set the desired goal that we want to reach
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = self._rooms_coord[room_index][0]
		goal.target_pose.pose.position.y = self._rooms_coord[room_index][1]
		goal.target_pose.pose.orientation.w = 1
		# Sends the goal to the action server.
		self.move_cli.send_goal(goal)
		
	
	def check_motion(self):
		""" 
		This method checks if the planner has done its execution when the state of the action service
		is equal to DONE.
		When it is done, the random path of via points going from the current point to the target point
		is retrieved. This path is then used by the controller.
		
		Args:
			self: instance of the current class.
		
		"""
		# Execute only when the plan action service is done
		if self.move_cli.get_state() == DONE and self.battery_low == False:
			log_msg = f'The ROBOT has arrived to the destination'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			# Update the position of the ROBOT in the ontology
			ARGS = ['isIn', 'Robot1', self._next_goal, self._prev_goal]
			ontology_manager('REPLACE', 'OBJECTPROP', 'IND' , ARGS)
			self._prev_goal = self._next_goal
			# Reason about the onoloy
			ARGS = ['']
			ontology_manager('REASON', '', '', ARGS)
			# Retreive the last time the robot moved
			ARGS = ['now', 'Robot1']
			last_motion = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
			last_motion = ontology_format(last_motion, 1, 11)
			# Retreive the last time a specific location has been visited
			ARGS = ['visitedAt', self._next_goal]
			last_location = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
			last_location = ontology_format(last_location, 1, 11) 
			# Update the time
			self.timer_now = str(int(time.time())) 
			# Update the timestamp since the robot moved
			ARGS = ['now', 'Robot1', 'Long', self.timer_now, last_motion[0]]
			ontology_manager('REPLACE', 'DATAPROP', 'IND', ARGS)
			# Update the timestamp since the robot visited the location
			ARGS = ['visitedAt', self._next_goal, 'Long', self.timer_now, last_location[0]]
			ontology_manager('REPLACE', 'DATAPROP', 'IND', ARGS)
			# Cancel the goal to be sure
			self.cancel_motion()
			self.motion_completed = True  # Set to True only the one involved in the state
	
		
	def motion_done(self):
		""" 
		Get the value of the variable responsible of stating the status of the planner.
		The returning value will be `True` if the planner has finished, `False` otherwise.
		
		Args:
			self: instance of the current class.
		
		Returns:
			self.plan_completed: Bool value that states the status of the planner.
		
		"""
		return self.motion_completed
		
	
	def cancel_motion(self):
		""" 
		This function executes the controller for a surveillance task. It starts by 
		getting the via points from the planner and sends a request to the ControlGoal() 
		action service in order to follow the desired path. 
		
		Args:
			self: instance of the current class.
		
		"""
		log_msg = f'Cancel the previous goal\n\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Cancel the goal
		self.move_cli.cancel_all_goals()
		rospy.sleep(1)
				
				
	def reset_var(self):
		""" 
		It is used to reset all the variables used to decide when a task has finished its execution.
		
		Args:
			self: instance of the current class.
			
		"""
		self.mutex.acquire()    # take the mutex
		self.reasoner_done = False
		self.motion_completed = False
		self.charge_reached = False
		self.check_completed = False
		self.mutex.release()    # release the mutex
		
	
	def do_surveillance(self):
		""" 
		It simulates a survaillance task of the location in which the robot arrives when the 
		controller has done its execution. 
		While it explores the location, also the status of the battery is checked.
		
		
		Args:
			self: instance of the current class.
		
		"""
		# Reset the boolean variables
		self.reset_var()
		# Surveillance task
		log_msg = f'The ROBOT starts surveilling room: {self._next_goal}\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		joint_angle = 0
		while self.battery_low == False and joint_angle <= 3.1: # If bettery low there won't be surveillance task
			# Rotate the base joint of the robot
			self.pub_base_joint.publish(joint_angle)
			joint_angle += 0.1
			self.rate.sleep()
		# Make the camera look the fron side of the robot
		joint_angle = 0
		self.pub_base_joint.publish(joint_angle)
		if self.battery_low == False:
			log_msg = f'The robot checked location: {self._next_goal}\n\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			self.check_completed = True  # Set to True only the one involved in the state
		else:
			log_msg = f'Stop surveilling! Battery low!\n\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			self.check_completed = False  # Set to False since action was not completed
	
	
	
	def surveillance_done(self):
		""" 
		Get the value of the variable responsible of stating the status of the surveillance task.
		The returning value will be `True` if the robot has checked the location, `False` otherwise.
		
		Args:
			self: instance of the current class.
		
		Returns:
			self.check_completed: Bool value that states the status of the surveillance task.
		
		"""
		return self.check_completed
		
