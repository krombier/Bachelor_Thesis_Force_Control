# Hybrid force/impedance control
### Detailed documentation can be found in the code. 
The impedance control part was made by Curdin Deplazes. Check out his [GitHub repo](https://github.com/CurdinDeplazes/cartesian_impedance_control) for further details about the impedance control or the following start up functions:
- on_init()
- on_configure()
- on_activate()
- on_deactivate()


Launch the controller: <br />
```bash
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py
```

Launch the impedance client if you want to change the position or adjust the impedance parameters (this part is currently commented out): <br />
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

## Input service

The service consists of a server and a client. The following files are part of the service: user_input_client.cpp, user_input_server.cpp and user_input_client.hpp. The client has to be called in a second terminal, while the server is automatically launched when launching the controller. The client can be started with:
``` bash
ros2 run cartesian_impedance_control force_control_client 
```

### Input server

The input server consists of a main function and one callback function per client. At first, the node is created, and then one server per changeable parameter. As of now, the pose, impedance parameter and mode have an own server. As soon as a client with the same service name is called, a callback function is executed, where the sent request is assigned to the parameter running on the controller.

### Input client
The input client consists of one main function only. At first rclcpp::init(argc,argv) is called to initialize communication. Then, the node on which all different clients are created on. After that, a request for all the different parameters one wants to change. Next, using a std::cin statement, the desired parameters to change are read in by the program, assigned to the respective parts of the request, the request is sent and the node spined until a successful response is received. When the server is stopped, the node is shut down. If it is desired to change a new parameter, this first needs to be given into the constructor of the UserInputServer so that the correct parameter can be changed during runtime. Then a client and a server have to be implemented in the same way as already inside the respective files. In addition to that, if required new messages have to be defined.

## Force service
Eventhough the program asks you to choose the frame in which you would like to exert forces, only the base frame ( frame =1 ) does work.  
The client has to be called in a second terminal, while the server is automatically launched when launching the controller.  
The client to set forces gets created either when running the user_input_client or when running the force_control_client. If you want to use the force_control_client make sure to comment the corresponding part in the user_input_client. (Not sure if it will cause problems, but better be safe)
To start the client use the following command:
``` bash
ros2 run cartesian_impedance_control force_control_client 
```

### Force server
Consists of a force_control_server.cpp and a force_control_server.hpp file.
Creates a server to exert requested forces.

### Force client
Consists of a force_control_client.cpp file. 
The function printframe() tells the user the frame in which he is currently operating.  
First the main function asks the user to choose a frame in which he would like to work.  
Then the user can choose between two modes:  
1. A few hardcoded commands between which the user can switch.  
2. You can repetetly enter a wrench of your choosing in the base frame of the robot in the format (F_x, F_y, F_z, M_x, M_y, M_z)
