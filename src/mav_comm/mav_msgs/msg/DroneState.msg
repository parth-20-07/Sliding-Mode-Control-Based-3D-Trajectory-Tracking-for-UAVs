Header header

# This message defines the drone state.

# We use the coordinate frames with the following convention:
#   x: forward
#   y: left
#   z: up

geometry_msgs/Vector3 position 					                        # Drone position [m].
geometry_msgs/Vector3 linear_velocity                           # Drone linear velocity [m/s].
geometry_msgs/Vector3 linear_acceleration                       # Drone linear acceleration [m/s^2].

geometry_msgs/Quaternion orientation                            # Drone orientation expressed in quaternions.
geometry_msgs/Vector3 angular_velocity                          # Drone angular velocity [rad/s].
geometry_msgs/Vector3 angular_acceleration                      # Drone angular acceleration [rad/s^2].
