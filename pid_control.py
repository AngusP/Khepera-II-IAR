class PID_control:



    distance_threshold = [400]*8
    distance_error = [0]*8
    distance_error_sum = [0]*8

    Kp = [0.5]*8
    Ki = [0.3]*8
    Kd = [0.2]*8

########################################################################
    def find_collisions(self, sensors):
        for sensor_index in xrange(7):
            if sensors[sensor_index] > distance_threshold[sensor_index]:
                yield True
########################################################################
    def pid_distance(self, sensors):
        estimated_distances = [0]*8 # 0 standing for infinite distance
        for sensor_index in xrange(7):

            threshold = distance_threshold[sensor_index]
            value = sensors[sensor_index]

            #calculate changes in error
            error = value - threshold
            error_sum = distance_error_sum[sensor_index] + error
            error_difference = error - distance_error[sensor_index] 

            #record new error values
            distance_error[sensor_index] = error 
            distance_error_sum[sensor_index] = error_sum

            #return the PID-estimated measurements
            estimated_distances[sensor_index] = Kp*error + Ki*error_sum + kd*error_difference

        return  estimated_distances
#########################################################################
    def front_stop(self, distance):
        if distance > distance_threshold[2]:
            drive(0,0)
        
            
            
    



        
        

