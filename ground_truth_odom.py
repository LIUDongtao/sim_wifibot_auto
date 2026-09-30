#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


POSE_TOPIC = "/world/wifibot_indoor_world/dynamic_pose/info"


class GroundTruthOdom(Node):

    def __init__(self):
        super().__init__(
    "ground_truth_odom",
    parameter_overrides=[
        rclpy.parameter.Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
    ]
)

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            TFMessage,
            POSE_TOPIC,
            self.pose_callback,
            10
        )

        self.get_logger().info(
            "Ground-truth odometry started."
        )

    def pose_callback(self, msg):

        # ros_gz_bridge converts Gazebo Pose_V into TFMessage.
        # Find the transform corresponding to the Wifibot model.
        robot_tf = None

        for transform in msg.transforms:

            name = transform.child_frame_id

            if name == "wifibot" or name.endswith("/wifibot"):
                robot_tf = transform
                break

        if robot_tf is None:
            return

        now = self.get_clock().now().to_msg()

        # -------------------------------
        # Publish nav_msgs/Odometry
        # -------------------------------

        odom = Odometry()

        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = \
            robot_tf.transform.translation.x

        odom.pose.pose.position.y = \
            robot_tf.transform.translation.y

        # base_footprint should be on the ground plane
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = \
            robot_tf.transform.rotation

        # Small non-zero covariance
        odom.pose.covariance[0] = 0.0001
        odom.pose.covariance[7] = 0.0001
        odom.pose.covariance[35] = 0.0001

        self.odom_pub.publish(odom)

        # -------------------------------
        # Publish odom -> base_footprint TF
        # -------------------------------

        tf = TransformStamped()

        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"

        tf.transform.translation.x = \
            robot_tf.transform.translation.x

        tf.transform.translation.y = \
            robot_tf.transform.translation.y

        tf.transform.translation.z = 0.0

        tf.transform.rotation = \
            robot_tf.transform.rotation

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):

    rclpy.init(args=args)

    node = GroundTruthOdom()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
