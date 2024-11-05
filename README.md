# Hybrid Force/Impedance Controller

### ROS2 `cartesian_impedance_controller` from Pd|Z

This repository contains three ROS2 packages.

## Package Overview

1. **Hybrid Force/Impedance Controller**  
   **Name**: `cartesian_impedance_control`  
   This is the main package where users can interact with and control the system.

2. **Custom Messages for the Controller**  
   **Name**: `messages_fr3`  
   This package provides the custom messages required by the hybrid force/impedance controller.  
   **Note**: The controller will not function without these messages.

3. **Wrench Plotter**  
   **Name**: `realtime_plot_of_data`  
   An optional package for real-time plotting of wrenches.  
   **Note**: If you don't need this feature, simply delete the package folder after installation.

Details about each package can be found in the code and individual `README` files within each package directory.

## Prerequisites

Ensure you have the following installed:

- **ROS2 Humble**
- **Libfranka 0.13.0** or newer
- **franka_ros2 v0.13.1**

For more information, refer to the [Franka ROS2 FCI documentation](https://support.franka.de/docs/franka_ros2.html).

## Installation Instructions

1. **Clone the Repository**  
   Clone this repository into the `src` directory of your ROS2 workspace (e.g., `franka_ros2_ws`):
   ```bash
   cd franka_ros2_ws/src  # Go to the root of your workspace
   git clone https://github.com/CurdinDeplazes/cartesian_impedance_control.git

2. **If you already have a src folder**, ensure all files are consolidated into a single src directory and delete any duplicate folders.

3. Add the following lines of code, to your **controllers.yaml** file inside franka_ros2/franka_bringup/config/:
```bash
cartesian_impedance_controller:
      type: cartesian_impedance_control/CartesianImpedanceController
```

Build the packages or whole workspace: <br />
```bash
colcon build --packages-select cartesian_impedance_control messages_fr3 realtime_plot_of_data --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release #Builds all the packages in your src folder
```

If not yet done, ensure your setup is always sourced by adding the following line to the end of your .bashrc file (to get access to it, you need to execute `nano .bashrc` in your home directory). : <br />
```bash
source /home/<user>/franka_ros2_ws/install/setup.sh 
```

Launch the controller: <br />
```bash
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py
```

Launch the client if you want to adjust parameters: <br />
``` bash
ros2 run cartesian_impedance_control user_input_client 
```

Usefull links:
https://frankaemika.github.io/libfranka/0.14.1/structfranka_1_1Errors.html Name: Explaination of the errors returned by the controller.
https://frankaemika.github.io/libfranka/0.14.1/structfranka_1_1RobotState.html Name: List of all states of the robot that can be accessed.
https://frankaemika.github.io/libfranka/0.14.1/index.html Name: Libfranka documentation (the sections classes and files contain tons of examples and information)
