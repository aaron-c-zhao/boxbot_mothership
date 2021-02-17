search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=boxbot.srdf
robot_name_in_srdf=boxbot
moveit_config_pkg=boxbot_moveit_config
robot_name=boxbot
planning_group_name=arm
ikfast_plugin_pkg=boxbot_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=wrist_link
ikfast_output_path=/home/aaronzhao/Project/boxbot_mothership/rcs/catkin_ws/src/boxbot_arm_ikfast_plugin/src/boxbot_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
