float32 dy_in_m                        # target - distance to closest point [m]. negative means go left. 
float32 dtheta_in_rad                  # angle by which the robot's orientation should be fixed [rad]. negative means clockwise. 
float32 lin_vel                        # linear velocity at which the robot should go [m/s]

float32 theta_of_closest_point         # 1st measurement: angle of closest point from the x-axis of the robot [rad]
float32 range_of_closest_point         # 2nd measurement: range of closest point from the robot [m]

