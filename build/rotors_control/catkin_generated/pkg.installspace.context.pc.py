# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;mav_msgs;nav_msgs;roscpp;sensor_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-llee_position_controller;-lposition_controller;-lcrazyflie_onboard_controller;-lroll_pitch_yawrate_thrust_controller;-lsensfusion6;-lcrazyflie_complementary_filter;-lmellinger_controller;-linternal_model_controller;-lroll_pitch_yawrate_thrust_controller_crazyflie".split(';') if "-llee_position_controller;-lposition_controller;-lcrazyflie_onboard_controller;-lroll_pitch_yawrate_thrust_controller;-lsensfusion6;-lcrazyflie_complementary_filter;-lmellinger_controller;-linternal_model_controller;-lroll_pitch_yawrate_thrust_controller_crazyflie" != "" else []
PROJECT_NAME = "rotors_control"
PROJECT_SPACE_DIR = "/home/prar/rbe502_project/install"
PROJECT_VERSION = "7.0.1"
