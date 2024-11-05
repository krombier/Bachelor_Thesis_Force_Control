# Hybrid force/impedance control
### ROS2 catestian_impedance_controller from Pd|Z



Launch the controller: <br />
```bash
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py
```

Launch the impedance client if you want to change the position or adjust the impedance parameters: <br />
``` bash
ros2 run cartesian_impedance_control user_input_client 
```

Launch the force client if you want to exert forces : <br />
``` bash
ros2 run cartesian_impedance_control force_control_client 
```

Launch the gripper control node if you want to steer the gripper with : <br />
``` bash
ros2 run cartesian_impedance_control gripper 
```

     
