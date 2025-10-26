import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import matplotlib.pyplot as plt
from collections import deque

class PIDPlotter(Node):
    def __init__(self, mode):
        super().__init__('pid_plotter')
        self.mode = mode  # 设置绘制模式
        self.max_length = 3000  # 数据的最大长度

        if self.mode == 1:
            # 一维模式
            self.subscription = self.create_subscription(
                Float32MultiArray,
                'pid_1d',
                self.listener_callback_1d,
                10)
            self.error_data = deque(maxlen=self.max_length)

        elif self.mode == 2:
            # 二维模式
            self.subscription = self.create_subscription(
                Float32MultiArray,
                'pid_2d',
                self.listener_callback_2d,
                10)
            self.error_data_x = deque(maxlen=self.max_length)
            self.error_data_y = deque(maxlen=self.max_length)

        elif self.mode == 3:
            # 三维模式
            self.subscription = self.create_subscription(
                Float32MultiArray,
                'pid_3d',
                self.listener_callback_3d,
                10)
            self.error_data_x = deque(maxlen=self.max_length)
            self.error_data_y = deque(maxlen=self.max_length)
            self.error_data_z = deque(maxlen=self.max_length)

        self.time_data = deque(maxlen=self.max_length)

        plt.ion() 
        self.fig = plt.figure()
        if self.mode == 1:
            self.ax = self.fig.add_subplot(111)
            self.line, = self.ax.plot([], [], 'r-', label='error')

        elif self.mode == 2:
            self.ax = self.fig.add_subplot(111)
            self.line_x, = self.ax.plot([], [], 'r-', label='x')
            self.line_y, = self.ax.plot([], [], 'b-', label='y')

        elif self.mode == 3:
            from mpl_toolkits.mplot3d import Axes3D
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.line, = self.ax.plot([], [], [], 'r-', label='error')

        self.ax.set_xlim(0, self.max_length)
        self.ax.set_ylim(-5, 5)

        if self.mode == 3:
            self.ax.set_zlim(-5, 5)

        plt.legend()
        plt.show()

        self.counter = 0  # 用于保持时间序列

    def listener_callback_1d(self, msg):
        if len(msg.data) != 1:
            self.get_logger().warn("Expected 2 elements in data array")
            return
         
        self.get_logger().info(f'Received: {msg.data}')
        self.error_data.append(msg.data[0])
        self.time_data.append(self.counter)
        self.counter += 1
        self.update_plot()

    def listener_callback_2d(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn("Expected 2 elements in data array")
            return

        self.get_logger().info(f'Received: {msg.data}')
        self.error_data_x.append(msg.data[0])
        self.error_data_y.append(msg.data[1])
        self.time_data.append(self.counter)
        self.counter += 1
        self.update_plot()

    def listener_callback_3d(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warn("Expected 3 elements in data array")
            return

        self.get_logger().info(f'Received: {msg.data}')
        self.error_data_x.append(msg.data[0])
        self.error_data_y.append(msg.data[1])
        self.error_data_z.append(msg.data[2])
        self.time_data.append(self.counter)
        self.counter += 1
        self.update_plot()

    def update_plot(self):
        if self.mode == 1:
            self.line.set_data(self.time_data, self.error_data)
            if len(self.error_data) > 0:
                self.ax.set_xlim(self.time_data[0], self.time_data[-1])
                self.ax.set_ylim(min(self.error_data) - 1, max(self.error_data) + 1)

        elif self.mode == 2:
            self.line_x.set_data(self.time_data, self.error_data_x)
            self.line_y.set_data(self.time_data, self.error_data_y)
            if len(self.error_data_x) > 0 and len(self.error_data_y) > 0:
                self.ax.set_xlim(self.time_data[0], self.time_data[-1])
                self.ax.set_ylim(min(min(self.error_data_x), min(self.error_data_y)) - 1, 
                                 max(max(self.error_data_x), max(self.error_data_y)) + 1)

        elif self.mode == 3:
            self.line.set_data(self.time_data, self.error_data_x, self.error_data_y, self.error_data_z)
            if len(self.error_data_x) > 0 and len(self.error_data_y) > 0 and len(self.error_data_z) > 0:
                self.ax.set_xlim(self.time_data[0], self.time_data[-1])
                self.ax.set_ylim(min(min(self.error_data_x), min(self.error_data_y)) - 1, 
                                 max(max(self.error_data_x), max(self.error_data_y)) + 1)
                self.ax.set_zlim(min(self.error_data_z) - 1, max(self.error_data_z) + 1)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)

    mode = 2  
    pid_plotter = PIDPlotter(mode)

    try:
        rclpy.spin(pid_plotter)
    except KeyboardInterrupt:
        print("Shutting down")

    pid_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
