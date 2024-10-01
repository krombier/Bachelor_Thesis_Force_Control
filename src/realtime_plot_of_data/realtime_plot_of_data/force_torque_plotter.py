import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import matplotlib.pyplot as plt
from collections import deque
import time

# Name of Node ForceTorquePlotter
# topic subscribed to: /franka_robot_state_broadcaster/external_wrench_in_base_frame

# This Node takes the external forces and torques measured by the robot and plots them
class ForceTorquePlotter(Node):
    def __init__(self):
        super().__init__('force_torque_plotter')
        self.subscription = self.create_subscription(
            Wrench,                                                          # here we subscribe to the corresponding node
            'wrench_topic',        # here as well
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.force_data = deque(maxlen=1000)     #adapt those values to store more than 1000 values
        self.torque_data = deque(maxlen=1000)    #deque automatically gets rid of the last element ones it reaches it limit defined by maxlen
        self.time_data = deque(maxlen=1000)

        self.start_time = time.time()

        # Setting up the plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        force_labels = ['Force x', 'Force y', 'Force z']
        torque_labels = ['Torque x', 'Torque y', 'Torque z']
        self.force_lines = [self.ax1.plot([], [], label=force_labels[i])[0] for i in range(3)]
        self.torque_lines = [self.ax2.plot([], [], label=torque_labels[i])[0] for i in range(3)]
        self.ax1.set_title('Observed force vs Time')
        self.ax2.set_title('Observed torque vs Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax2.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Force (N)')
        self.ax2.set_ylabel('Torque (Nm)')
        self.ax1.legend()
        self.ax2.legend()
        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        #print("Print does work")
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        #print("hello")
        force = msg.force
        torque = msg.torque
        self.force_data.append([force.x, force.y, force.z])
        self.torque_data.append([torque.x, torque.y, torque.z])
        self.get_logger().info(f'Force data appended: {force.x}, {force.y}, {force.z}')
        self.get_logger().info(f'Torque data appended: {torque.x}, {torque.y}, {torque.z}')

        self.update_plot()

    def update_plot(self):
        force_data = list(zip(*self.force_data))
        torque_data = list(zip(*self.torque_data))
        time_data = list(self.time_data)

        for i in range(3):
            self.force_lines[i].set_data(time_data, force_data[i])
            self.torque_lines[i].set_data(time_data, torque_data[i])

        self.ax1.relim()        #recalculate the axis limits
        self.ax2.relim()
        self.ax1.autoscale_view()
        self.ax2.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    force_torque_plotter = ForceTorquePlotter()
    rclpy.spin(force_torque_plotter)
    force_torque_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
