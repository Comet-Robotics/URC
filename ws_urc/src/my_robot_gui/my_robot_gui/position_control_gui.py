import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk

class PositionControlGUI(Node):
    def __init__(self):
        super().__init__('position_control_gui')

        # Create a parameter to get the topic that we want to publish to
        self.declare_parameter('controller_topic', '/forward_position_controller/commands')
        controller_topic = self.get_parameter('controller_topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(
            Float64MultiArray, controller_topic, 10)

        # Create a parameter to set the number of joints
        self.declare_parameter('joint_count', 2)
        self.joint_count = self.get_parameter('joint_count').get_parameter_value().integer_value
        self.sliders = [] # List of slider widgets
        self.joint_values = [0.0] * self.joint_count  # Initialize joint values

        # Create Tkinter window
        self.window = tk.Tk()
        self.window.title("Joint Position Control")

        # Create sliders for each joint
        self.declare_parameter('range', ['-3.14', '3.14']) # Parameter to controller the range of the Slider, default is -pi to pi
        range_min = float(self.get_parameter('range').get_parameter_value().string_array_value[0])
        range_max = float(self.get_parameter('range').get_parameter_value().string_array_value[1])
        for i in range(self.joint_count):
            label = ttk.Label(self.window, text=f"Joint {i+1}:")
            label.grid(row=i, column=0, padx=5, pady=5)

            slider = tk.Scale(self.window, from_=range_min, to=range_max,
                                resolution=0.01, orient=tk.HORIZONTAL,
                                command=lambda value, index=i: self.update_joint_value(index, value))
            slider.grid(row=i, column=1, padx=5, pady=5)
            self.sliders.append(slider)

        # Publish button
        publish_button = ttk.Button(self.window, text="Publish", command=self.publish_joint_positions)
        publish_button.grid(row=self.joint_count, column=0, columnspan=2, padx=5, pady=10)

    # Callback to update joint values when sliders are moved
    def update_joint_value(self, index, value):
        self.joint_values[index] = float(value)

    # Callback to publish the positions of the sliders to the controller topic
    def publish_joint_positions(self):
        msg = Float64MultiArray()
        msg.data = self.joint_values
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


    def main_loop(self):
        self.window.mainloop()


def main(args=None):
    rclpy.init(args=args)
    position_control_gui = PositionControlGUI()
    position_control_gui.main_loop()  # Run Tkinter's main loop
    rclpy.shutdown()

if __name__ == '__main__':
    main()