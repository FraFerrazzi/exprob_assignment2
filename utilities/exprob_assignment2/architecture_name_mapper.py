#!/usr/bin/env python
import rospy

# The name of the parameter to define the environment size.
# It should be a list `[max_x, max_y]` such that x:[0, `max_x`) and y:[0, `max_y`).
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# Define the location in which the robot starts
INIT_LOCATION = 'E'

# Define the location in which the robot goes to recharge itself.
CHARGE_LOCATION = 'E'

# Define the starting points in which the robot starts
INIT_POINT = [0.0, 0.0]

# Initialize and define the size of the environment 
ENVIRONMENT_SIZE = [20, 15]

# The boolean parameter to active random testing.
# If the value is `False` a keyboard-based interface will be used to produce stimulus 
# (i.e., battery signals). Instead, random stimulus will be generated if `True`. In the 
# latter case, the architecture also requires all the parameters with the scope 
# `test/random_sense/*`, which are not used if the boolean parameter is `False`.
PARAM_RANDOM_ACTIVE = 'test/random_sense/active'
# ---------------------------------------------------------


# The name of the node representing the shared knowledge required for this scenario.
NODE_ROBOT_BATTERY_STATE = 'robot-battery-state'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# The name of the service solving the recharge of the robot.
TOPIC_RECHARGE = 'state/recharge'

# The delay for the battery to become low, i.e., from high to low.
# It should be a list `[min_time, max_time]`, and the battery level change
# will occur after a random number of seconds within such an interval.
PARAM_BATTERY_TIME = 'test/random_sense/battery_time'

# The delay of the charging time regarding the battery, i.e., from low to high.
# It should be a list `[min_time, max_time]`, and the battery will
# be charged after a random number of seconds within such an interval.
PARAM_BATTERY_CHARGE = 'test/random_sense/battery_charge'
# ---------------------------------------------------------


# The name of the node representing the shared knowledge required for this scenario.
NODE_STATE_MACHINE = 'state-machine'

# The name of the service solving the initialization of the world.
TOPIC_WORLD = '/world_init'
# ---------------------------------------------------------


# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# Defines the number of points in the plan. It is a list where the number of points
# is a random value in the interval [`min_n`, `max_n`).
PARAM_PLANNER_POINTS = 'test/random_plan_points'

# The delay between the computation of the next via points.
# It should be a list `[min_time, max_time]`, and the computation will 
# last for a random number of seconds in such an interval.
PARAM_PLANNER_TIME = 'test/random_plan_time'
# -------------------------------------------------


# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The time required to reach a via point given by the planner.
# It is a list `[min_time, max_time]`, and the time to reach a
# via point will be a random number of seconds in such an interval.
PARAM_CONTROLLER_TIME = 'test/random_motion_time'
# -------------------------------------------------


# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return '@%s>> %s' % (producer_tag, msg)
