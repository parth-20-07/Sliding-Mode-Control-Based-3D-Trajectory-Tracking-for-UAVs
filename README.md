# Sliding_mode_controller_for_quadrotor

sliding mode controller is designed for altitude and attitude control of the Crazyflie 2.0 to enable the
quadrotor to track desired trajectories and visit a set of desired waypoints.

Desired set of waaypoints are given by :

  • p 0 = (0, 0, 0) to p 1 = (0, 0, 1) in 5 seconds
  • p 1 = (0, 0, 1) to p 2 = (1, 0, 1) in 15 seconds
  • p 2 = (1, 0, 1) to p 3 = (1, 1, 1) in 15 seconds
  • p 3 = (1, 1, 1) to p 4 = (0, 1, 1) in 15 seconds
  • p 4 = (0, 1, 1) to p 5 = (0, 0, 1) in 15 seconds
#PRAR
