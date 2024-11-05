# Hybrid force/impedance control



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

Launch the gripper control node if you want to steer the gripper with the user_input_client: <br />
``` bash
ros2 run cartesian_impedance_control gripper 
```
(To steer the gripper you need to use the user_input_client file, this one just takes care of the commands sent out)
