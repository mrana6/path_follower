# path_follower

## Functionalities
The package currently allows loading cartesian-space (converted to joint-space using hlpr_trac_ik) or joint-space trajectory and executing it using veclociy or position control (using wpi_jaco_wrapper)

## Dependencies
1. kinova-ros packages (at GT-RAIL (7dof branch))
2. (Optional: if using on nimbus) nimbus_bot packages (at GT-RAIL)
3. (Optional: for cartesian playback) hlpr_trac_ik (at HLP-R/hlpr_manipulation/hlpr_trac_ik)

## Launching
1. Place the trajectory to executed in `.txt` file in either `data_cartesian` or `data_joints` directory depending on the type of trajectory you have.
2. Launch the trajectory server. For nimbus: `roslaunch nimbus_bringup nimbus_bringup.launch`.
3. (Optional: for cartesian playback) Launch HLPR-trac-ik: `roslaunch hlpr_trac_ik start_ik_service.launch base_chain:=<base_link_name> end_chain:=<ee_link_name>`
4. Run path follower: `roslaunch path_follower start_path_follower.launch arm_prefix:=<jaco_/right_> control_mode:=<velocity/position> data_mode:=<cartesian/joints> data_name:=<file_name>.txt`

## WARNING
1. To avoid collisions, move the arm to somewhere close to the init in the trajectory.
2. In `follow_path.py` the argument `seconds` in function `execute_trajectory` sets the time between waypoints. It has been set to `0.02` by default. Change if required.

