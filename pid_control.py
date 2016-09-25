import math

class PID_control:



    distance_threshold = [100]*8
    distance_error = [0]*8
    distance_error_sum = [0]*8

    Kp = [1.0]*8
    Ki = [0.0]*8
    Kd = [0.0]*8

########################################################################

    def pid_distance(self, sensors):
        control_variable = [0]*8 # 0 standing for infinite distance
        for sensor_index, sensor_val in enumerate(sensors):
            threshold = self.distance_threshold[sensor_index]
            value = sensor_val

            #calculate changes in error
            error = value - threshold
            error_sum = self.distance_error_sum[sensor_index] + error
            error_difference = error - self.distance_error[sensor_index] 

            #record new error values
            self.distance_error[sensor_index] = error 
            self.distance_error_sum[sensor_index] = error_sum

            #return the PID-estimated measurements
            control_variable[sensor_index] = self.Kp[sensor_index]*error + self.Ki[sensor_index]*error_sum + self.Kd[sensor_index]*error_difference

        return  control_variable

#########################################################################
    def pid_force_vector(self, control_variable):
	
	result = [0.0,0.0]
	distance_vectors = [[0.0,0.0]]*8 # (x,y) format, x horzontal, y vertical wrt. robot

	if(self.distance_error[0] > self.distance_threshold[0]): 
		distance_vectors[0] = [-control_variable[0], 0]

	if(self.distance_error[1] > self.distance_threshold[1]): 
		distance_vectors[1] = [-control_variable[1]*math.cos(math.pi/4), control_variable[1]*math.sin(math.pi/4)]

	if(self.distance_error[2] > self.distance_threshold[2]): 
		distance_vectors[2] = [0, control_variable[2]]

	if(self.distance_error[3] > self.distance_threshold[3]): 
		distance_vectors[3] = [0, control_variable[3]]

	if(self.distance_error[4] > self.distance_threshold[4]): 
		distance_vectors[4] = [control_variable[4]*math.cos(math.pi/4), control_variable[4]*math.sin(math.pi/4)]

	if(self.distance_error[5] > self.distance_threshold[5]): 
		distance_vectors[5] = [control_variable[5], 0]

	if(self.distance_error[6] > self.distance_threshold[6]): 
		distance_vectors[6] = [0, -control_variable[6]]

	if(self.distance_error[7] > self.distance_threshold[7]): 
		distance_vectors[7] = [0, -control_variable[7]]
	
	for vector in distance_vectors:
		result[0] = vector[0] + result[0]
		result[1] = vector[1] + result[1]
		
	
	print result

	return result



	
	

#########################################################################

        
        

