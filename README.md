markdown

# Hybrid Force/Impedance Controller

### ROS2 `cartesian_impedance_controller` from Pd|Z

This repository contains three ROS2 packages aimed at facilitating the development and use of a hybrid force/impedance controller for robotic applications.

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

If you already have a src folder, ensure all files are consolidated into a single src directory and delete any duplicate folders.

    Update controllers.yaml
    Add the following lines to your controllers.yaml file located in franka_ros2/franka_bringup/config/:

    yaml

cartesian_impedance_controller:
  type: cartesian_impedance_control/CartesianImpedanceController

Build the Workspace
Build the selected packages or the entire workspace:

bash

colcon build --packages-select cartesian_impedance_control messages_fr3 realtime_plot_of_data --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  # Builds all packages in your src folder

Source the Workspace
Ensure your setup is sourced by adding the following line to the end of your .bashrc file:

bash

source /home/<user>/franka_ros2_ws/install/setup.sh

To edit your .bashrc, use:

bash

    nano ~/.bashrc

Launching the Controller

To launch the controller, run:

bash

ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py

Adjusting Parameters

To launch the client for parameter adjustment, run:

bash

ros2 run cartesian_impedance_control user_input_client

Useful Links

    Explanation of Controller Errors
    List of Robot States
    Libfranka Documentation
    (Contains detailed examples and information in the Classes and Files sections)

Feel free to explore and contribute to this repository. If you have any questions or issues, please open an issue or submit a pull request!

vbnet


You can copy this directly to your `README.md` file, and the formatting will be preserve
