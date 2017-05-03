# path_follower

# Functionalities
The package currently allows loading cartesian-space (converted to joint-space using hlpr_trac_ik) or joint-space trajectory and executing it using veclociy or position control (using wpi_jaco_wrapper)

# Dependencies
1. wpi_jaco_wrapper (pull the latest version on develop branch)
2. jaco2_description (at gt-rail_internal/codebase/davidkent/jaco2_description)
3. jaco2_bringup (clone from here)
4. hlpr_trac_ik (at HLP-R/hlpr_manipulation/hlpr_trac_ik)
5. rail_generalized_cylinder (clone from here)

# Launching
1. `roslaunch jaco2_bringup jaco2_bringup.launch`
2. `roslaunch hlpr_trac_ik start_ik_service.launch base_chain:=<base_link_name> end_chain:=<ee_link_name>`
3. `roslaunch rail_generalized_cylinder start_path_follower.launch arm_prefix:=<jaco_/right_> control_mode:=<velocity/position> data_mode:=<cartesian/joints> data_name:=<file_name>.txt`

