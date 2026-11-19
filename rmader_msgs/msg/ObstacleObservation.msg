# One observation of an obstacle whose future motion is unknown (e.g. from a
# tracker or a motion capture system). Published on /obstacle_observations
# and consumed by rmader_node when dyn_obstacles/enabled is true.
Header header                  # header.stamp = time at which the obstacle was at pos (planner's ROS clock; 0 = on arrival)
int32 id                       # unique per obstacle, stable across observations
geometry_msgs/Vector3 pos      # centre of the obstacle [m], world frame
geometry_msgs/Vector3 vel      # velocity [m/s]; only used if has_vel is true
bool has_vel                   # false: the planner estimates the velocity itself
geometry_msgs/Vector3 bbox     # full size of the obstacle's bounding box [m]
bool is_static                 # true: known not to move (a wall, a parked crate). Its box never grows and the
                               # track is kept without being observed again
