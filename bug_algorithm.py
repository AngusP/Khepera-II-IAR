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

class Bug_Algorithm:


	def lineMagnitude (self, x1, y1, x2, y2):
	    lineMagnitude = math.sqrt(math.pow((x2 - x1), 2)+ math.pow((y2 - y1), 2))
	    return lineMagnitude
	 
	#Calc minimum distance from a point and a line segment (so if too far to reach it, we start a new heading, so we do not go in a full loop around the box)
	def DistancePointLine (self, px, py, x1, y1, x2, y2):
	 
	    LineMag = self.lineMagnitude(x1, y1, x2, y2)
	 
	    if LineMag < 0.00000001:
		DistancePointLine = 9999
		return DistancePointLine
	 
	    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
	    u = u1 / (LineMag * LineMag)
	 
	    if (u < 0.00001) or (u > 1):
		#// closest point does not fall within the line segment, take the shorter distance
		#// to an endpoint
		ix = self.lineMagnitude(px, py, x1, y1)
		iy = self.lineMagnitude(px, py, x2, y2)

		if ix > iy:
		    DistancePointLine = iy
		else:
		    DistancePointLine = ix
	    else:
		# Intersecting point is on the line, use the formula
		ix = x1 + u * (x2 - x1)
		iy = y1 + u * (y2 - y1)
		DistancePointLine = self.lineMagnitude(px, py, ix, iy)
	 
	    return DistancePointLine	


	#check if c is between a and b
	def is_on_mline(self,a, b, c):

	    #thresholded cross product to account for non-ideal odometry
	    crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1])
	    if abs(crossproduct) > 20 : return False   
	    
	    #thresholded dot product to account for non-ideal odometry
	    dotproduct = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1])*(b[1] - a[1])
	    if dotproduct < -20 : return False


	    #thresholded to account for non-ideal odometry
	    squaredlengthba = (b[0] - a[0])*(b[0] - a[0]) + (b[1] - a[1])*(b[1] - a[1])
	    if dotproduct > squaredlengthba + 20: return False

	    return True	

		
	#vector magnitude calculator
	def vector_magnitude(self, vector):
		return math.sqrt( math.pow(vector[0],2) + math.pow(vector[1],2))
		

	#normalizes angle in degrees to -180 : 180 degrees
        def normalize_angle(self, angle):
		if angle < 0:
		    angle = angle % -360
		    if angle < -180:
			angle = 360 + angle
		else:
		    angle = angle % 360
		    if angle > 180:
			angle = -(360 - angle)
		return angle

	#vector difference calculator
	def vector_diff(self, vector_1, vector_2):
		dx = vector_1[0] - vector_2[0]*1.0
		dy = vector_1[1] - vector_2[1]*1.0
		return [dx,dy]

		 
	#get angle while ON the M-line
	def get_angle_on_m(self, odo_state, bug_state):
		direction = self.vector_diff( bug_state.m_line_start,  bug_state.m_line_end)
		direction_angle = math.atan2(direction[1] , direction[0])
		#if no difference, well then we never left the spot 
		vector_magnitude = self.vector_magnitude(direction)
		if vector_magnitude == 0:
			return 0
		
		direction_angle = math.degrees(direction_angle) - self.normalize_angle(math.degrees(odo_state.theta))
		return self.normalize_angle(direction_angle)
		 	
	#return new state
        def new_state(self, nav_state, odo_state, bug_state):
	                
	    
            	speed_l = nav_state.speed_l 
	    	speed_r = nav_state.speed_r

		#do not turn if not needed

		turn_less = constants.CONST_SPEED 
		turn_more = constants.CONST_SPEED

		
		#set up more convenient variables for checking distances, use integers to account for rounding errors
		current_pos = [int(odo_state.x), int(odo_state.y)]
		previous_pos = [int(bug_state.last_m_x) , int(bug_state.last_m_y)]
		line_start = [ int(bug_state.m_line_start[0]) , int(bug_state.m_line_start[1]) ]

		#if we are on the mline again
		on_mline = self.is_on_mline(line_start, previous_pos, current_pos)
		closer_on_mline = False

		#new distance to the origin
		new_distance = self.vector_magnitude(current_pos)
		old_distance =  self.vector_magnitude(previous_pos)


		#distance to M-line (more accurately)
		dist_to_mline = self.DistancePointLine(odo_state.x , odo_state.y, bug_state.m_line_start[0] , bug_state.m_line_start[1] , bug_state.m_line_end[0], bug_state.m_line_end[1])

		on_mline =  dist_to_mline <= constants.ON_MLINE or on_mline
		

		#check if we reached the destination
		if new_distance < 40:
			bug_state.done = True
		
		#if far away from M-line, different constants for free space and wall following
		far_in_open_space = dist_to_mline > constants.M_DISTANCE * 4.0 and nav_state.system_state == constants.STATE_DRIVE_FORWARD
		far_on_wall = dist_to_mline > constants.M_DISTANCE*10.0
		#recalculate M-line if such a need arises
		if far_in_open_space or far_on_wall:
			
			#renew m-line record
			bug_state.m_line_end  = [odo_state.x , odo_state.y]
			bug_state.m_line_start= [0,0]

			#update last position on line
			bug_state.last_m_x = odo_state.x
   			bug_state.last_m_y = odo_state.y
			
			#turn aggressively
			turn_less = -constants.CONST_SPEED 
			turn_more = constants.CONST_SPEED 

		#Try keeping the robot on the M-line (be it old one or updated one)
		if True:
			#initially assume we are not following a wall
			is_wall_following = False
			#then assign it a value, for clarity in this case
			is_wall_following = nav_state.system_state == constants.STATE_RIGHT_FOLLOW and nav_state.system_state == constants.STATE_LEFT_FOLLOW
			#If are in free space
			if nav_state.system_state == constants.STATE_DRIVE_FORWARD:

				turn_less = -constants.CONST_SPEED 
				turn_more = constants.CONST_SPEED 

				#record new closer position on m-line if we are getting closer along it
				if on_mline and new_distance <= old_distance:
					bug_state.last_m_x = odo_state.x
   					bug_state.last_m_y = odo_state.y
					

			# if can leave a wall closer on m-line
			elif new_distance <= old_distance and is_wall_following and on_mline:

				#turn aggressively
				turn_less = -constants.CONST_SPEED 
				turn_more = constants.CONST_SPEED

				#make this the new closest we have gotten along m-line
				bug_state.last_m_x = odo_state.x
   				bug_state.last_m_y = odo_state.y 
				closer_on_mline = True

			
			angle_to_m = self.get_angle_on_m(odo_state, bug_state)

			#OUR angle too small
			if angle_to_m > constants.M_N_ANGLE:		
				
				#if are following a wall and are closer along m-line to goal
				#or are not wall following (are in free space)
				if closer_on_mline or not is_wall_following:
					# check if there is a wall on the left
					if nav_state.system_state is not constants.STATE_LEFT_FOLLOW:
						# if there is none, turn left
						speed_r = turn_more
						speed_l = turn_less	

			#OUR angle too big
			elif angle_to_m < -constants.M_N_ANGLE:


				#if are following a wall and are closer along m-line to goal
				#or are not wall following (are in free space)
				if closer_on_mline or not is_wall_following:
					# check if there is a wall on the right
					if nav_state.system_state is not constants.STATE_RIGHT_FOLLOW:
						# if there is none, turn right
						speed_r = turn_less
						speed_l = turn_more

		#send out the new speed controls			
		nav_state.speed_l = speed_l
		nav_state.speed_r = speed_r 
		return nav_state
		    
