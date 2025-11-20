#!/usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np
import random
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleTargetPlotter(Node):
    def __init__(self):
        super().__init__('turtle_target_plotter')
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.update_plot)
        self.x = 5.0
        self.y = 5.0
        self.theta = 0.0
        self.target_x = random.uniform(0, 9)
        self.target_y = random.uniform(0, 9)
        self.target_size = 1.0
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.setup_plot()

    def setup_plot(self):
        self.ax.set_xlim(0, 11)
        self.ax.set_ylim(0, 11)
        self.ax.set_xticks(np.arange(0, 11, 1))
        self.ax.set_yticks(np.arange(0, 11, 1))
        self.ax.grid(True)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Turtle1 Pose with Random Target')

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        if (self.target_x <= self.x <= self.target_x + self.target_size) and (self.target_y <= self.y <= self.target_y + self.target_size):
            self.generate_new_target()

    def generate_new_target(self):
        self.target_x = random.uniform(0, 9)
        self.target_y = random.uniform(0, 9)

    def update_plot(self):
        self.ax.cla()
        self.setup_plot()
        target_rect = plt.Rectangle((self.target_x, self.target_y), self.target_size, self.target_size, color='green', alpha=0.4)
        self.ax.add_patch(target_rect)
        arrow_scale = 0.5
        dx = np.cos(self.theta) * arrow_scale
        dy = np.sin(self.theta) * arrow_scale
        self.ax.arrow(self.x, self.y, dx, dy, head_width=0.2, head_length=0.25, fc='blue', ec='blue', linewidth=2)
        self.ax.plot(self.x, self.y, 'ro')
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTargetPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.ioff()
        plt.show()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
