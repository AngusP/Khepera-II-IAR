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

	#function to determine if algorithm permits exploration
	def can_explore(self, result, pathing_state):

	    #key point - maximum exploration until we find the 1st food source
	    #determine if we are following a wall
	    wall_following = result.system_state == constants.STATE_RIGHT_FOLLOW or result.system_state == constants.STATE_LEFT_FOLLOW
	    #determine if have not explored this round yet and done with main part of algorithm

	    #TODO uncomment the food bit maybe
	    should_explore_pathing = pathing_state.exploration_not_done() and not pathing_state.has_uncollected_food() 

	    #if pathing not activated, we do not care if we have completed the main portion of the algorithm
	    exploration_beneficial = should_explore_pathing or not pathing_state.algorithm_activated
	    #return result that if we are wall following and if we would benefit from exploration and are already not doing so
	    return wall_following and exploration_beneficial and not result.boredom

        def new_state(self, nav_state, pathing_state, comms,ds):
	                
	    result = Navigation_State()
	    result = nav_state 



            ############################
            # HANDLE BOREDOM
            ############################

	    #TODO check if traiviling off in a random direction is more useful potentially (or make it collect only after it's done with food)
	    if self.can_explore(result, pathing_state):
			#keep coubnt of round we followed a wall
			result.boredom_counter += 1
			if result.boredom_counter > constants.MAX_BOREDOM:

				#drive away from the wall we were hugging
				if result.system_state == constants.STATE_LEFT_FOLLOW:
					comms.drive(constants.CONST_SPEED, -constants.CONST_SPEED)
				else:
					comms.drive(-constants.CONST_SPEED, constants.CONST_SPEED)

				# do for number of rounds that fit in 1 second
				for x in xrange(1.0 / constants.MEASUREMENT_PERIOD_MS):
					odo_state = odo.new_state(odo_state, comms.get_odo())
  	    				nav_state.dist = comms.get_ir()
         				ds.push(odo_state, nav_state.dist)
					#sleep to let sensors detect stuff
					time.sleep(constants.MEASUREMENT_PERIOD_MS)

				#drive forwards until stuck
				comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)
				print "BORED"
				#make the boredom driving forward happen, reset boredom counter
				result.boredom = True
				result.boredom_counter = 0
				
			
				

 	    #variable to indicate if reactive avoidance is in control of the robot
	    result.yielding_control = False
		

            ############################
            # IF STUCK
            ############################

            if self.is_stuck(result.dist): 

		 if result.boredom:
			print "DONE BEING BORED"

		 #disable boredom
		 result.boredom = False
		 #make sure to denote exploration end on this round
		 pathing_state.complete_exploration()
		 #make sure do not continue spiralling into the wall
		 pathing_state.end_spiral()

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


            ############################
            # IF BOREDOM
            ############################

	    #if boredom active, continue driving until we get stuck
            if result.boredom:
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
		    
