int32                    node_id               # the id of the node

geometry_msgs/Point32    p                     # the location of the node in world coordinates  [in m]

int32                    degree                # the degree of the node (for endpoints 1, for meetpoints # of obstacles)

float64                  closest_distance      # closest distance to the obstacle(s) that defined the endpoint (meetpoint) [in m]

float64[]                edge_angle_diffs      # the angles between the edges adjacent to this node [in deg]
                                               # for example at a T-shaped meetpoint the edges are at (-90, 90, 180)
                                               # or (identically) (-180, -90, 90). In this case the edge_angle_diffs would be (90, 180, 90)

float64[]                possible_bearings     # the possible bearings that the robot can take at this node [in deg]
			 		       # for meetpoints it does not include the backwards direction. for a T-shaped
					       # meetpoint it is (-90, 90) if it arrives from the south   

laser_node/Obstacles      surrounding_obstacles   # obstacles that are visible the first time this node is created    
                                                               
