# Hybrid Force/Impedance controller
### ROS2 catestian_impedance_controller from Pd|Z

This GitHub repo contains 3 ROS2 packages:

1. A hybrid force/impedance controller     Name: cartesian_impedance_control
2. Custom messages that are needed for the hybrid force impedance controller Name: messages_fr3
3. A wrench plotter Name: realtime_plot_of_data

The first package is the one where you can acctually fo things.
Fat: The hybrid force/impedance controller does not work without the custom messages.
The third package is not needed unless you want to plot a wrench in real time.

Details about the packages can be found in the code and in the readme files within the packages.


Prerequisites:
* ROS2 humble <br />
* Libfranka 0.13.0 or newer <br />
* franka_ros2 v0.13.1 <br />

For further information, please refer to the [Franka ROS2 FCI documentation](https://support.franka.de/docs/franka_ros2.html)

Clone this repository in the src directory of your ROS 2 workspace (in this example we call it franka_ros2_ws): <br />
```bash
cd franka_ros2_ws/src             #go to the root of your workspace
git clone https://github.com/CurdinDeplazes/cartesian_impedance_control.git
```
If you already have a src folder, make sure to move all files/folders into 1 src folder and delete the other one.

For the moment, you need to add the following lines of code, to your controllers.yaml file inside franka_ros2/franka_bringup/config/:
```bash
cartesian_impedance_controller:
      type: cartesian_impedance_control/CartesianImpedanceController
```

Build the packages or whole workspace: <br />
```bash
colcon build --packages-select cartesian_impedance_control messages_fr3 realtime_plot_of_data --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release #Builds all the packages in your src folder
```

If not yet done, ensure your setup is always source by adding the following line to the end of your .bashrc file (to get access to it, you need to execute `nano .bashrc` in your home directory). : <br />
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

Common problems:

