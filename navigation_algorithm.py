#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from navigation_state import Navigation_State

import constants
import sys

import time
import math

class Navigation_Algorithm:


	# check if we are stuck
	def is_stuck(self, dist):
	    #check if we are scraping on the sides
	    # multiple of 1.2 as 1.0 is handled by following
	    stuck_cone_left  = dist[1] > constants.CONST_WALL_DIST 
	    # multiple of 1.2 as 1.0 is handled by following
	    stuck_cone_right = dist[4] > constants.CONST_WALL_DIST 
	    #check if we are about to be stuck in the front
	    stuck_cone_front = dist[2] > constants.CONST_WALL_DIST*0.5 or dist[3]  > constants.CONST_WALL_DIST*0.5

	    return stuck_cone_left or stuck_cone_right or stuck_cone_front 


	# check if we "see" the left wall and it is closer than the one on the right
	def should_follow_left_wall(self, dist, system_state):
	    within_range =  dist[0] > constants.CONST_WALL_DIST * 0.5
	    #to not switch wall if two walls nearby
	    followed_right = system_state == constants.STATE_RIGHT_FOLLOW
	    return within_range and dist[0] > dist[5] and not followed_right
		

	# check if we "see" the right wall and it is closer than the one on the left
	def should_follow_right_wall(self, dist, system_state):
	    within_range =  dist[5] > constants.CONST_WALL_DIST * 0.5
	    followed_left = system_state == constants.STATE_LEFT_FOLLOW
	    return within_range and not(dist[0] > dist[5]) and not followed_left

	# check if we are over the distance threshold w.r.t. object on the left
	def too_close_to_left(self, dist):

	    distance_close   = dist[0] > constants.CONST_WALL_DIST
	    return distance_close 


	# check if we are over the distance threshold w.r.t. object on the right
	def too_close_to_right(self, dist):

	    distance_close   = dist[5] > constants.CONST_WALL_DIST
	    return distance_close 



	# check if we are under the distance threshold w.r.t. object on the right
	def is_away_from_right(self, dist):

	    wall_in_range = dist[5] < constants.CONST_WALL_DIST * 0.8
	    return wall_in_range


	# check if we are under the distance threshold w.r.t. object on the left
	def is_away_from_left(self, dist):

	    wall_in_range = dist[0] < constants.CONST_WALL_DIST * 0.8
	    return wall_in_range


	# check if there is more space on the right of the robot than on the left
	def is_more_space_on_right(self, dist):

	    values_on_right = dist[3] + dist[4] + dist[5]
	    values_on_left  = dist[1] + dist[2] + dist[0]

	    return (values_on_left > values_on_right) 


	# check if the system is being unstuck
	def is_being_unstuck(self, state):

	    return (state is constants.STATE_STUCK_LEFT) or (state is constants.STATE_STUCK_RIGHT)


	# check if it is more beneficial to unstuck by turning to the right
	def should_unstuck_right(self, dist, system_state):
	    stuck_on_left = system_state == constants.STATE_LEFT_FOLLOW 
	    return stuck_on_left or self.is_more_space_on_right(dist)

	# check if robot is "bored"
	def bored(self, boredom_counter):
	    return boredom_counter >= constants.CONST_WALL_BORED_MAX

	# check if we are handling boredom
	def is_boredom_handled(self, state):
	    return state == constants.STATE_BOREDOM_ROTATE or state == constants.STATE_BOREDOM_DRIVE


        def new_state(self, nav_state):
	                
	    result = Navigation_State()
	    result = nav_state


 	    #variable to indicate if reactive avoidance is in control of the robot
	    result.yielding_control = False

  		#TODO consider boredom as a harmful concept 

            ############################
            # IF STUCK
            ############################

            if self.is_stuck(result.dist):      

                 # do not interrupt if already handle
                 if self.is_being_unstuck(result.system_state):
			return result
		
                 # determine direction of where better to turn to unstuck
                 if self.should_unstuck_right(result.dist, result.system_state):

                      result.system_state = constants.STATE_STUCK_RIGHT
		      result.speed_l = constants.CONST_SPEED
		      result.speed_r = -constants.CONST_SPEED		

                 else:

                      result.system_state = constants.STATE_STUCK_LEFT
		      result.speed_l = -constants.CONST_SPEED
		      result.speed_r = constants.CONST_SPEED
                
		 return result

            #####################
            ##WALL FOLLOWING LEFT
            #####################

	    #if not stuck 
            result.yielding_control = True

            if self.should_follow_left_wall(result.dist, result.system_state):

		turn_least = constants.CONST_SPEED * constants.TURN_LESS
		turn_most  = constants.CONST_SPEED * constants.TURN_MORE
		no_turn    = constants.CONST_SPEED
		speed_r = result.speed_r
		speed_l = result.speed_l

                # set state accordingly 
                result.system_state = constants.STATE_LEFT_FOLLOW

                # keep the distance within the threshold range
                if self.too_close_to_left(result.dist) and not (speed_l == turn_most and speed_r == turn_least):

                    speed_l = turn_most
                    speed_r = turn_least

                elif self.is_away_from_left(result.dist) and not (speed_l == turn_least and speed_r == turn_most):

                    speed_l = turn_least
                    speed_r = turn_most

                elif not (speed_l == no_turn and speed_r == no_turn): 

                    speed_l = no_turn
                    speed_r = no_turn

 		result.speed_r = speed_r
                result.speed_l = speed_l

            #####################
            ##WALL FOLLOWING RIGHT
            #####################                       
            elif self.should_follow_right_wall(result.dist, result.system_state):

                #print("following right")

		turn_least = constants.CONST_SPEED * constants.TURN_LESS
		turn_most  = constants.CONST_SPEED * constants.TURN_MORE
		no_turn    = constants.CONST_SPEED
		speed_r = result.speed_r
		speed_l = result.speed_l

                # set state accordingly
                result.system_state = constants.STATE_RIGHT_FOLLOW

                # keep the distance within the threshold range 
                if self.too_close_to_right(result.dist) and not (speed_l == turn_least and speed_r == turn_most):

                    speed_l = turn_least 
                    speed_r = turn_most 

                elif self.is_away_from_right(result.dist) and not (speed_l == turn_most and speed_r == turn_least):

                    speed_l = turn_most
                    speed_r = turn_least

                elif not (speed_l == no_turn and speed_r == no_turn): 

                    speed_l = no_turn
                    speed_r = no_turn

 		result.speed_r = speed_r
                result.speed_l = speed_l

            ######################
            # IF NONE OF THE ABOVE
            #####################
            else:
                    # reset variables as not doing anything
                    result.boredom_counter = 0

                    # set state accordingly
                    result.system_state = constants.STATE_DRIVE_FORWARD
	            result.speed_l = constants.CONST_SPEED
		    result.speed_r = constants.CONST_SPEED



	    return result  
		    
