Obstacle[]   collection
Obstacle[]   merged_collection
int32        countScans          # the number of laser scans that were segmented             
uint32 	     seq                 # the unique id of this message  
time         stamp               # Timestamp for visualization
float64      angle_min           # the leftmost angle the laser can scan [in rad]
float64      angle_max           # the rightmost angle the laser can scan [in rad] 
float64      angle_increment     # the angle difference between consecutive scans [in rad]
