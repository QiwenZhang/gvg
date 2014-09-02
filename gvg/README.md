gvg
===

Generalized Voronoi Graph autonomous indoor navigation


ROS Parameters
==============

laser_offset_x, laser_offset_y: Set the distance offset of the laser range finder on the robot. e.g.: 0,0 offset if laser is centered on robot
leave_meetpoint_min_dist, leave_meetpoint_max_dist: Min/max distance that the robot will travel after orienting itself towards the chosen edge (when leaving a meetpoint)

meetpoint_threshold: Distance from calculated meetpoint before we consider that the robot reached meetpoint. 
do_move: Move/stop the robot.
lin_vel: Constant linear velocity.
ang_vel: Constant rotational velocity. [rad]

relocalize_epsilon: Variable to define the random chance of relocalizing.
closest_distance_threshold: Minimum distance that must be respected between meetpoints.
exploration_policy: 0 for random walk, 1 for ear-based exploration.

vertex_matching_policy: 0 for odometry-based/full automation (no confirmation with user input), 1 for fully manual control (confirmation with user input for meetpoint decisions), 2 for vertex matching policy.
edge_length_threshold: Threshold for edge length comparison (part of vertex matching policy)

reverse_laser: Set to "true" only if you need the remapper node to reverse the laser scan. Only use this if your laser range finder is installed upside down.

robot_diam: Robot's diameter. For safety reasons, you should always choose the longer side as the diameter if your robot is not circular.
laser_max_range: Max range capability of your laser range finder.

dy_signal, dy_kp, dy_ki, dy_kd: Initial signal for PID controller and PID gains for the robot/GVG-line error.
ang_vel_signal, ang_vel_kp, ang_vel_ki, ang_vel_kd: signal for PID controller and PID gains for the robot orientation error.


ROS Topics
==========
General topics:

  /indoor/gvg/odom_combined: Output odometry from the localizer
  /indoor/laser_utils/closest_obstacles: Topic published by laser_node, gives all closest obstacles in current scan
  /indoor/follow_wall: GVG navigation topic
  /indoor/gvg/theMap: Published map topic
  
Simulation-specific:

  /indoor/cmd_vel: Output commands to the simulated robot
  
  /indoor/base_scan: Input laser range finder scans from Stage
  /indoor/odom: Odometry topic from Stage
  
Robot-specific:

  /<robot_name>/cmd_vel: Output commands to the robot
  /indoor/pose_to_odom: Remapper functionality that takes in PoseWithCovarianceStamped msgs and re-publishes as Odometry msgs
  /robot_pose_ekf/odom: Read in PoseWithCovarianceStamped topic from the robot_pose_ekf node (ROS default node)
  /lidar/scan: Input laser scan from laser range finder
  /lidar/reversed: Output laser scan by remapper node with the scans inverted (because our robot has lidar installed upside down)

