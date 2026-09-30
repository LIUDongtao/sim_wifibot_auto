import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelAdapter(Node):

    def __init__(self):
        super().__init__('cmd_vel_adapter')

        # 接收 path_follower.py 的速度指令
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        # 发送给 Ignition Gazebo 的差速驱动
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info(
            'Wifibot cmd_vel adapter started: '
            '/cmd_vel_raw -> /cmd_vel'
        )

    def cmd_callback(self, msg):
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelAdapter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # 退出时发送停车命令
    stop_msg = Twist()
    node.cmd_pub.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

