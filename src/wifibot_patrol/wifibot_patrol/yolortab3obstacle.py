#!/usr/bin/env python3
import argparse, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformException
from ultralytics import YOLO

class YoloGazeboObstacle(Node):
    def __init__(self, args):
        super().__init__("yolo_gazebo_obstacle")
        self.model = YOLO(args.model)
        self.bridge = CvBridge()
        self.conf = args.conf
        self.max_obstacles = args.max_obstacles
        self.target_frame = args.frame_id
        # Pinhole back-projection gives ROS optical coordinates:
        # X right, Y down, Z forward.
        self.camera_frame = args.camera_frame

        self.depth = None
        self.depth_encoding = ""
        self.fx = self.fy = self.cx = self.cy = None
        self.seen_rgb = self.seen_depth = self.seen_info = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self.pose_pub = self.create_publisher(PoseArray, args.pose_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, args.cloud_topic, 10)

        self.create_subscription(Image, args.image_topic, self.image_cb, 10)
        self.create_subscription(Image, args.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.info_cb, 10)

        self.get_logger().info("YOLO Gazebo obstacle node started")
        self.get_logger().info(f"RGB: {args.image_topic}")
        self.get_logger().info(f"Depth: {args.depth_topic}")
        self.get_logger().info(f"CameraInfo: {args.camera_info_topic}")
        self.get_logger().info(f"3D source frame: {self.camera_frame}")
        self.get_logger().info(f"Target frame: {self.target_frame}")

    def info_cb(self, msg):
        self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
        self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
        if not self.seen_info:
            self.seen_info = True
            self.get_logger().info(
                f"[YOLO] CameraInfo received fx={self.fx:.2f} fy={self.fy:.2f} "
                f"cx={self.cx:.2f} cy={self.cy:.2f}")

    def depth_cb(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_encoding = msg.encoding
            if not self.seen_depth:
                self.seen_depth = True
                self.get_logger().info(
                    f"[YOLO] Depth received {self.depth.shape}, "
                    f"encoding={msg.encoding}, frame={msg.header.frame_id}")
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    def depth_at(self, u, v):
        if self.depth is None:
            return None
        h, w = self.depth.shape[:2]
        u, v = int(np.clip(u, 0, w-1)), int(np.clip(v, 0, h-1))
        r = 4
        a = self.depth[max(0,v-r):min(h,v+r+1),
                       max(0,u-r):min(w,u+r+1)].astype(np.float32)
        a = a[np.isfinite(a)]
        a = a[a > 0]
        if a.size == 0:
            return None
        d = float(np.median(a))
        if self.depth_encoding == "16UC1" or d > 100.0:
            d /= 1000.0
        if not math.isfinite(d) or d < 0.05 or d > 20.0:
            return None
        return d

    @staticmethod
    def qrotate(v, q):
        x,y,z = v
        qx,qy,qz,qw = q
        uv = (qy*z-qz*y, qz*x-qx*z, qx*y-qy*x)
        uuv = (qy*uv[2]-qz*uv[1],
               qz*uv[0]-qx*uv[2],
               qx*uv[1]-qy*uv[0])
        return tuple(v[i] + 2.0*(qw*uv[i] + uuv[i]) for i in range(3))

    def to_target(self, x, y, z):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f"TF unavailable {self.target_frame} <- {self.camera_frame}: {e}",
                throttle_duration_sec=2.0)
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        rx,ry,rz = self.qrotate((x,y,z), (q.x,q.y,q.z,q.w))
        return rx+t.x, ry+t.y, rz+t.z

    @staticmethod
    def cloud(points, frame, stamp):
        h = Header()
        h.frame_id, h.stamp = frame, stamp
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        a = np.asarray(points, dtype=np.float32).reshape((-1,3)) if points else np.empty((0,3), np.float32)
        return PointCloud2(header=h, height=1, width=len(a), fields=fields,
                           is_bigendian=False, point_step=12, row_step=12*len(a),
                           data=a.tobytes(), is_dense=False)

    def image_cb(self, msg):
        if not self.seen_rgb:
            self.seen_rgb = True
            self.get_logger().info(
                f"[YOLO] RGB received {msg.width}x{msg.height}, "
                f"encoding={msg.encoding}, frame={msg.header.frame_id}")
        if self.depth is None or self.fx is None:
            self.get_logger().warn("Waiting for Depth + CameraInfo...",
                                   throttle_duration_sec=2.0)
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            results = self.model(img, verbose=False, conf=self.conf)
        except Exception as e:
            self.get_logger().error(f"YOLO processing failed: {e}")
            return

        objects = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1,y1,x2,y2 = box.xyxy[0].cpu().numpy().tolist()
                u,v = int((x1+x2)/2), int((y1+y2)/2)
                d = self.depth_at(u,v)
                if d is None:
                    continue
                # optical frame: X right, Y down, Z forward
                cx = (u-self.cx)*d/self.fx
                cy = (v-self.cy)*d/self.fy
                cz = d
                p = self.to_target(cx,cy,cz)
                if p is None:
                    continue
                name = self.model.names.get(cls_id, f"class_{cls_id}")
                objects.append((d, name, conf, (cx,cy,cz), p))

        objects.sort(key=lambda x: x[0])
        objects = objects[:self.max_obstacles]
        stamp = self.get_clock().now().to_msg()

        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        pa = PoseArray()
        pa.header.frame_id, pa.header.stamp = self.target_frame, stamp
        points = []

        for i,(d,name,conf,cam,p) in enumerate(objects):
            x,y,z = p
            points.append((x,y,z))

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x,y,z
            pose.orientation.w = 1.0
            pa.poses.append(pose)

            m = Marker()
            m.header.frame_id, m.header.stamp = self.target_frame, stamp
            m.ns, m.id, m.type, m.action = "yolo_points", i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x,y,z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.18
            m.color.r, m.color.a = 1.0, 1.0
            ma.markers.append(m)

            txt = Marker()
            txt.header.frame_id, txt.header.stamp = self.target_frame, stamp
            txt.ns, txt.id = "yolo_labels", 1000+i
            txt.type, txt.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x,y,z+0.35
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.25
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f"{name}\n{conf*100:.1f}%\n{d:.2f} m"
            ma.markers.append(txt)

            self.get_logger().info(
                f"[YOLO] {name} conf={conf:.2f} depth={d:.2f}m "
                f"camera=({cam[0]:.2f},{cam[1]:.2f},{cam[2]:.2f}) "
                f"map=({x:.2f},{y:.2f},{z:.2f})",
                throttle_duration_sec=0.5)

        self.marker_pub.publish(ma)
        self.pose_pub.publish(pa)
        self.cloud_pub.publish(self.cloud(points, self.target_frame, stamp))

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="yolo11n.pt")
    p.add_argument("--image_topic", default="/camera/zed2i/image_raw")
    p.add_argument("--depth_topic", default="/camera/zed2i/depth/image_raw")
    p.add_argument("--camera_info_topic", default="/camera/zed2i/camera_info")
    p.add_argument("--marker_topic", default="/yolo/obstacle_markers")
    p.add_argument("--pose_topic", default="/yolo/obstacle_poses")
    p.add_argument("--cloud_topic", default="/rtabmap/cloud_obstacles")
    p.add_argument("--frame_id", default="map")
    p.add_argument("--camera_frame", default="camera_optical_frame")
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--max_obstacles", type=int, default=3)
    args = p.parse_args()

    rclpy.init()
    node = YoloGazeboObstacle(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
