class PID_control:



    distance_threshold = [100]*8
    distance_error = [0]*8
    distance_error_sum = [0]*8

    Kp = [0.9]*8
    Ki = [0.0]*8
    Kd = [0.0]*8

########################################################################
    def find_collisions(self, sensors):
        for sensor_index in xrange(7):
            if sensors[sensor_index] > self.distance_threshold[sensor_index]:
                yield True

########################################################################
    def pid_distance(self, sensors):
        estimated_distances = [0]*8 # 0 standing for infinite distance
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
            estimated_distances[sensor_index] = self.Kp[sensor_index]*error + self.Ki[sensor_index]*error_sum + self.Kd[sensor_index]*error_difference

        return  estimated_distances

#########################################################################
    def front_stop(self, distance):
        if distance > self.distance_threshold[2]:
            drive(0,0)
        
        

