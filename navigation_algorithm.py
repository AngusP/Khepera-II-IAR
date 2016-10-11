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
	    stuck_cone_left  = dist[1] > constants.CONST_WALL_DIST * 1.2
	    # multiple of 1.2 as 1.0 is handled by following
	    stuck_cone_right = dist[4] > constants.CONST_WALL_DIST * 1.2
	    #check if we are about to be stuck in the front
	    stuck_cone_front = dist[2] > constants.CONST_WALL_DIST*0.7 or dist[3]  > constants.CONST_WALL_DIST*0.7
	
	    return stuck_cone_left or stuck_cone_right or stuck_cone_front 
	
	
	# check if we "see" the left wall and it is closer than the one on the right
	def should_follow_left_wall(self,dist):
	        
	    return dist[0] > constants.CONST_WALL_DIST and dist[0] > dist[5]
	        
	
	# check if we "see" the right wall and it is closer than the one on the left
	def should_follow_right_wall(self,dist):
	
	    return dist[5] > constants.CONST_WALL_DIST and dist[5] > dist[0]
	
	
	# check if we are over the distance threshold w.r.t. object on the left
	def too_close_to_left(self,dist):
	
	    distance_close   = dist[0] > constants.CONST_WALL_DIST
	    return distance_close 
	
	
	# check if we are over the distance threshold w.r.t. object on the right
	def too_close_to_right(self,dist):
	
	    distance_close   = dist[5] > constants.CONST_WALL_DIST
	    return distance_close 
	
	
	
	# check if we are under the distance threshold w.r.t. object on the right
	def is_away_from_right(self,dist):
	
	    wall_in_range = dist[5] < constants.CONST_WALL_DIST - constants.CONST_WALL_OFFSET
	    return wall_in_range
	
	
	# check if we are under the distance threshold w.r.t. object on the left
	def is_away_from_left(self,dist):
	
	    wall_in_range = dist[0] < constants.CONST_WALL_DIST - constants.CONST_WALL_OFFSET
	    return wall_in_range
	
	
	
	# check if we no longer "see" the object on the right
	def is_right_wall_lost(self,dist): 
	
	    return dist[5] < constants.CONST_INF_DIST
	
	
	# check if we no longer "see" the object on the left
	def is_left_wall_lost(self,dist): 
	    
	    return dist[0] < constants.CONST_INF_DIST
	
	
	# check if there is more space on the right of the robot than on the left
	def is_more_space_on_right(self,dist):
	
	    values_on_right = dist[3] + dist[4] + dist[5]
	    values_on_left  = dist[1] + dist[2] + dist[0]
	
	    return (values_on_left > values_on_right) 
	
	
	# check if the system is being unstuck
	def is_being_unstuck(self,state):
	
	    return (state is constants.STATE_STUCK_LEFT) or (state is constants.STATE_STUCK_RIGHT)
	
	
	# check if it is more beneficial to unstuck by turning to the right
	def should_unstuck_right(self, dist, wall_is_followed_left):
	
	    no_preference_decision = (not wall_is_followed_left) and self.is_more_space_on_right(dist)
	    return wall_is_followed_left or no_preference_decision
	
	# check if robot is "bored"
	def bored(self, wall_boredom_counter):
	    return wall_boredom_counter >= constants.CONST_WALL_BORED_MAX
	
	# check if we are handling boredom
	def is_boredom_handled(self, state):
	    return state == constants.STATE_BOREDOM_ROTATE or state == constants.STATE_BOREDOM_DRIVE
	    
	    
	    
	def new_state(odo_state, nav_state):    
            
            #make it so the old values do not change unless needed
            result = nav_state
            
            ########################
            #HANDLE BOREDOM COUNTER 
            ########################
            
            #record that we are still moving along a wall and not "exploring"
            if nav_state.follow_left or nav_state.follow_right:
                result.boredom_counter +=1

            
            #check if robot is "bored" and it is not being handled
            if self.bored(nav_state.boredom_counter) and not self.is_boredom_handled(nav_state.system_state):

                print("bored...")

                #turn away from the wall that was last followed
                if nav_state.follow_left:
                    result.speed_l =  constants.CONST_SPEED 
                    result.speed_r = -constants.CONST_SPEED
                elif nav_state.follow_right:
                	result.speed_r =  constants.CONST_SPEED 
                    result.speed_l = -constants.CONST_SPEED
                
                # reset state
                result.boredom_turn_counter = 0
                result.boredom_counter = 0
                result.follow_left = False
                result.follow_right = False
                        
                # set state
                result.system_state = constants.STATE_BOREDOM_ROTATE
                #go to next iteration of the loop
                continue


            # if we are rotating on the spot                  
            if nav_state.system_state == constants.STATE_BOREDOM_ROTATE:
                #check if we are done rotating
                if nav_state.boredom_turn_counter >= constants.CONST_BORED_TURN_MAX:
                    #different from normal forward driving as we try to get very far from the wall
                    result.system_state = constants.STATE_BOREDOM_DRIVE
                    result.speed_r 		=  constants.CONST_SPEED 
                    result.speed_l 		=  constants.CONST_SPEED

                # otherwise, continue rotationg for this loop iteration
                else:
                    result.boredom_turn_counter += 1
                    #go to next iteration of the loop
                    continue

            ############################
            # IF STUCK
            ############################

            if self.is_stuck(nav_state.dist):
                #TODO do not forget to update sensor readings in these states and drive at returned speeds
                print("stuck")

                # do not interrupt if already handle
                if self.is_being_unstuck(nav_state.system_state):
                        continue

                # determine direction of where better to turn to unstuck
                if self.should_unstuck_right(nav_state.dist, nav_state.wall_is_followed_left):

                    result.system_state =  constants.STATE_STUCK_RIGHT
                    result.speed_r 		= -constants.CONST_SPEED 
                    result.speed_l 		=  constants.CONST_SPEED
                    comms.drive(constants.CONST_SPEED,-constants.CONST_SPEED)

                else:
                	
                    result.system_state =  constants.STATE_STUCK_LEFT
                    result.speed_r 		=  constants.CONST_SPEED 
                    result.speed_l 		= -constants.CONST_SPEED
                

                # stop following the wall (as it could ahve potentially led to being stuck)
                result.wall_is_followed_left = False
                result.wall_is_followed_right = False
                result.boredom_counter = 0


            # if robot not stuck and we are driving away from a "boring" wall, continue doing so
            elif nav_state.system_state == constants.STATE_BOREDOM_DRIVE:
                continue

            #####################
            ##WALL FOLLOWING LEFT
            #####################

            elif (nav_state.follow_left or self.should_follow_left_wall(nav_state.dist)) and not (self.is_left_wall_lost(nav_state.dist)):
                
                print("following left")

                # set state accordingly 
                result.follow_left = True
                result.follow_right = False
                result.system_state = constants.STATE_LEFT_FOLLOW

                # keep the distance within the threshold range
                if too_close_to_left(dist):

                    result.speed_l = constants.CONST_SPEED
                    result.speed_r = constants.CONST_SPEED * constants.CONST_TURN_PROPORTION

                elif is_away_from_left(dist):

                    result.speed_l = constants.CONST_SPEED * constants.CONST_TURN_PROPORTION
                    result.speed_r = constants.CONST_SPEED

                else: 

                    result.speed_l = constants.CONST_SPEED
                    result.speed_r = constants.CONST_SPEED

            #####################
            ##WALL FOLLOWING RIGHT
            #####################                       
            elif (nav_state.follow_right or self.should_follow_right_wall(nav_state.dist)) and not (self.is_right_wall_lost(nav_state.dist)):
                
                print("following right")

                # set state accordingly
                result.follow_left = False
                result.follow_right = True
                result.system_state = constants.STATE_RIGHT_FOLLOW

                # keep the distance within the threshold range 
                if too_close_to_right(dist):

                    result.speed_l = constants.CONST_SPEED * constants.CONST_TURN_PROPORTION
                    result.speed_r = constants.CONST_SPEED

                elif is_away_from_right(dist):

                    result.speed_l = constants.CONST_SPEED
                    result.speed_r = constants.CONST_SPEED * constants.CONST_TURN_PROPORTION

                else: 

                    result.speed_l = constants.CONST_SPEED
                    result.speed_r = constants.CONST_SPEED


            ######################
            # IF NONE OF THE ABOVE
            #####################
            else:

                print("driving...")

                # otherwise just drive forward
                if nav_state.system_state is not constants.STATE_DRIVE_FORWARD: 

                    # reset variables as not doing anything
                    result.follow_left 	 = False
                    result.follow_right  = False
                    result.boredom_counter = 0

                    # set state accordingly
                    result.system_state = constants.STATE_DRIVE_FORWARD
                    result.speed_l 		= constants.CONST_SPEED
                    result.speed_r 		= constants.CONST_SPEED    
	    
	    	return result
	    
	    
