int32	                 start_index      # scan index at which this object starts in the current laser scan  
int32   		 end_index        # scan index at which this object ends in the current laser scan

int32   		 min_index        # scan index of the closest point of this object to the laser
float64 		 min_distance     # distance of the closest object to the laser   [m]
float64 		 min_bearing      # bearing of the closest object in laser coordinates [rad] 
                                          # 0 if it is right in front of the robot, negative if it is on 
 					  # the right of the robot, positive if it is on the left

geometry_msgs/Point32	 closest_point    # the closest point on the obstacle in laser coordinates
geometry_msgs/Point32[]  surface          # all the points on this obstacle in laser coordinates 
