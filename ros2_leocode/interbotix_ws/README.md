## Commands to run the Interbotix Manipulator Control Node

- Clone the Repository and navigate to interbotix_ws in command line
- Perform a `colcon build` and `source install setup.bash` 
- Initialize the Robot Control node using the launch file:
``` ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=true ```
   (Note: the `use_sim` parameter can be set to `false` when the physical robot is in action)
This should launch the PincherX100 Robot model in Rviz
- Run the Manipulator control node in another terminal
  `ros2 run manip_control manip_control_node`
  (Refer to README in `manip_control` control package for functionalities of this node)
