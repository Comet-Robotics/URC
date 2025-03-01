import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk

class PositionControlGUI(Node):
    def __init__(self):
        super().__init__('position_control_gui')
        self.publisher_ = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.joint_count = 2  # Change this to the number of joints in your robot
        self.sliders = []
        self.joint_values = [0.0] * self.joint_count  # Initialize joint values

        # Create Tkinter window
        self.window = tk.Tk()
        self.window.title("Joint Position Control")

        # Create sliders for each joint
        for i in range(self.joint_count):
            label = ttk.Label(self.window, text=f"Joint {i+1}:")
            label.grid(row=i, column=0, padx=5, pady=5)

            slider = tk.Scale(self.window, from_=-3.14, to=3.14,  # Adjust range as needed
                                resolution=0.01, orient=tk.HORIZONTAL,
                                command=lambda value, index=i: self.update_joint_value(index, value))
            slider.grid(row=i, column=1, padx=5, pady=5)
            self.sliders.append(slider)

        # Publish button
        publish_button = ttk.Button(self.window, text="Publish", command=self.publish_joint_positions)
        publish_button.grid(row=self.joint_count, column=0, columnspan=2, padx=5, pady=10)

    def update_joint_value(self, index, value):
        self.joint_values[index] = float(value)

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