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
	 
	#Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).
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


	def vector_diff(self, vector_1, vector_2):
		dx = vector_1[0] - vector_2[0]*1.0
		dy = vector_1[1] - vector_2[1]*1.0
		return [dx,dy]


	def vector_magnitude(self, vector):
		return math.sqrt( math.pow(vector[0],2) + math.pow(vector[1],2))
	
	def normalize_vector(self, vector):

		vector_magnitude = self.vector_magnitude(vector)
		if vector_magnitude == 0:
			vector_magnitude = 1
		return (vector[0] / vector_magnitude, vector[1] / vector_magnitude)

		 
	#get angle while ON the M-line
	def get_angle_on_m(self, odo_state, bug_state):
		direction = self.vector_diff( bug_state.m_line_start,  bug_state.m_line_end)
		direction_angle = math.atan2(direction[1] , direction[0])
		#if no difference, well then we never left the spot 
		vector_magnitude = self.vector_magnitude(direction)
		if vector_magnitude == 0:
			return 0
		
		direction_angle = math.degrees(direction_angle) - self.normalize_angle(math.degrees(odo_state.theta))
		return direction_angle
		 	

        def new_state(self, nav_state, odo_state, bug_state):
	                
	    
            	speed_l = constants.CONST_SPEED
	    	speed_r = constants.CONST_SPEED
            	######################
            	# IF IN FREE SPACE, CAN FOLLOW M-LINE
            	#####################
		
		are_closer_than_before = False
		#check if we are closer on the m_x line
		current_pos = [odo_state.x, odo_state.y]
		previous_pos = [bug_state.last_m_x , bug_state.last_m_y]

		new_distance = self.vector_magnitude(current_pos)
		old_distance =  self.vector_magnitude(previous_pos)


		#distance to M-line
		dist_to_mline = self.DistancePointLine(odo_state.x , odo_state.y, bug_state.m_line_start[0] , bug_state.m_line_start[1] , bug_state.m_line_end[0], bug_state.m_line_end[1])
		


		
		#check if we reached the destination
		if new_distance < 40:
			bug_state.done = True
		
		#IF IN FREE ROAM
		print(dist_to_mline)
		#if far away from M-line
		if dist_to_mline > constants.M_DISTANCE and nav_state.system_state == constants.STATE_DRIVE_FORWARD:
			#DROP IT and for a new one as driving forward and just lost it due to collision or something
			
			bug_state.m_line_end  = [odo_state.x , odo_state.y]
			bug_state.m_line_start= [0,0]

			#update last position on lube
			bug_state.last_m_x = odo_state.x
   			bug_state.last_m_y = odo_state.y
			print("FUCK THIS M-LINE")

			angle_to_m = self.get_angle_on_m(odo_state, bug_state)
			#print "NOT NORMALIZED angle ON M-line %s" % (angle_to_m)	
			angle_to_m = self.normalize_angle(angle_to_m)

			#print "angle ON M-line %s" % (angle_to_m)

			# aggressive turning
			if nav_state.system_state == constants.STATE_DRIVE_FORWARD:
				turn_less = -constants.CONST_SPEED 
				turn_more = constants.CONST_SPEED 
				#OUR angle too small
				if angle_to_m > constants.M_N_ANGLE:
					speed_r = turn_more
					speed_l = turn_less
				#OUR angle too big
				elif angle_to_m < -constants.M_N_ANGLE:
					speed_r = turn_less
					speed_l = turn_more


		

		#if veering off M-line (but still on it)
		else:
			#TODO tweak
			turn_less = -constants.CONST_SPEED / 4
			turn_more = constants.CONST_SPEED 
			if new_distance < old_distance:
		

				print("OLD dist %s vs new Dist %s " % ( old_distance , new_distance ))
				bug_state.last_m_x = odo_state.x
   				bug_state.last_m_y = odo_state.y
				are_closer_than_before = True

				turn_less = constants.CONST_SPEED * constants.TURN_LESS
				turn_more = constants.CONST_SPEED * constants.TURN_MORE


			
			print(are_closer_than_before)
			angle_to_m = self.get_angle_on_m(odo_state, bug_state)
			#print "NOT NORMALIZED angle ON M-line %s" % (angle_to_m)	
			angle_to_m = self.normalize_angle(angle_to_m)

			#print "angle ON M-line %s" % (angle_to_m)

			# stay on the straight line
			if nav_state.system_state == constants.STATE_DRIVE_FORWARD:

				#OUR angle too small
				if angle_to_m > constants.M_N_ANGLE:
					speed_r = turn_more
					speed_l = turn_less
				#OUR angle too big
				elif angle_to_m < -constants.M_N_ANGLE:
					speed_r = turn_less
					speed_l = turn_more

			#TODO if are closer, then try leaving the wall towards the goal
			#else 

		nav_state.speed_l = speed_l
		nav_state.speed_r = speed_r 
		
		#print("BUG")

		return nav_state
		    
