#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FX = 909.8567
CAMERA_FY = 908.8925
CAMERA_CX = 647.8103
CAMERA_CY = 369.7298


@dataclass(frozen=True)
class VisualLink:
    link_name: str
    model_name: str
    sdf_xml: str


def _parse_xyz(text: str | None) -> tuple[float, float, float]:
    if not text:
        return 0.0, 0.0, 0.0
    values = [float(value) for value in text.split()]
    return values[0], values[1], values[2]


def _matrix_from_origin(origin: ET.Element | None) -> np.ndarray:
    xyz = _parse_xyz(origin.attrib.get("xyz") if origin is not None else None)
    rpy = _parse_xyz(origin.attrib.get("rpy") if origin is not None else None)
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    mat[:3, 3] = xyz
    return mat


def _pose_text_from_matrix(mat: np.ndarray) -> str:
    xyz = mat[:3, 3]
    rpy = Rotation.from_matrix(mat[:3, :3]).as_euler("xyz")
    return f"{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}"


def _safe_model_name(link_name: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_]", "_", link_name)
    return f"qpin_visual_{safe}"


def _resolve_mesh_uri(uri: str, description_dir: Path) -> str:
    package_prefix = "package://sailor_r1_pro_description/"
    if uri.startswith(package_prefix):
        return f"file://{description_dir}/{uri[len(package_prefix):]}"
    return uri


def _visual_material_xml(visual: ET.Element) -> str:
    color = visual.find("material/color")
    rgba = color.attrib.get("rgba", "1 1 1 1") if color is not None else "1 1 1 1"
    values = rgba.split()
    if len(values) != 4:
        values = ["1", "1", "1", "1"]
    rgba_text = " ".join(values)
    return (
        "<material>"
        f"<ambient>{rgba_text}</ambient>"
        f"<diffuse>{rgba_text}</diffuse>"
        "<specular>0.08 0.08 0.08 1</specular>"
        "<emissive>0 0 0 1</emissive>"
        "</material>"
    )


def _visual_geometry_xml(visual: ET.Element, description_dir: Path) -> str | None:
    mesh = visual.find("geometry/mesh")
    if mesh is not None:
        mesh_uri = mesh.attrib.get("filename", "")
        if not mesh_uri:
            return None
        return f"""
          <geometry>
            <mesh>
              <uri>{_resolve_mesh_uri(mesh_uri, description_dir)}</uri>
            </mesh>
          </geometry>"""

    box = visual.find("geometry/box")
    if box is not None:
        size = box.attrib.get("size", "").strip()
        if not size:
            return None
        return f"""
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>"""

    cylinder = visual.find("geometry/cylinder")
    if cylinder is not None:
        radius = cylinder.attrib.get("radius", "").strip()
        length = cylinder.attrib.get("length", "").strip()
        if not radius or not length:
            return None
        return f"""
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{length}</length>
            </cylinder>
          </geometry>"""

    return None


def _fixed_transform(root: ET.Element, parent_link: str, child_link: str) -> np.ndarray | None:
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        if parent.attrib.get("link") == parent_link and child.attrib.get("link") == child_link:
            return _matrix_from_origin(joint.find("origin"))
    return None


def _attached_camera_visuals(root: ET.Element, description_dir: Path) -> list[str]:
    body_from_optical = _fixed_transform(root, "body_link", "camera_color_optical_frame")
    optical_from_gazebo = _fixed_transform(root, "camera_color_optical_frame", "gazebo_camera_link")
    if body_from_optical is None or optical_from_gazebo is None:
        return []
    body_from_gazebo = body_from_optical @ optical_from_gazebo

    camera_link = None
    for link in root.findall("link"):
        if link.attrib.get("name") == "gazebo_camera_link":
            camera_link = link
            break
    if camera_link is None:
        return []

    visual_parts: list[str] = []
    for index, visual in enumerate(camera_link.findall("visual")):
        geometry = _visual_geometry_xml(visual, description_dir)
        if geometry is None:
            continue
        body_from_visual = body_from_gazebo @ _matrix_from_origin(visual.find("origin"))
        material = _visual_material_xml(visual)
        visual_parts.append(
            f"""
        <visual name="attached_gazebo_camera_link_{index}">
          <pose>{_pose_text_from_matrix(body_from_visual)}</pose>
          {geometry}
          {material}
        </visual>"""
        )
    return visual_parts


def _attached_camera_sensor(root: ET.Element) -> str | None:
    body_from_optical = _fixed_transform(root, "body_link", "camera_color_optical_frame")
    optical_from_gazebo = _fixed_transform(root, "camera_color_optical_frame", "gazebo_camera_link")
    if body_from_optical is None or optical_from_gazebo is None:
        return None
    body_from_gazebo = body_from_optical @ optical_from_gazebo
    hfov = 2.0 * math.atan(CAMERA_WIDTH / (2.0 * CAMERA_FX))
    return f"""
      <sensor name="camera_color" type="camera">
        <pose>{_pose_text_from_matrix(body_from_gazebo)}</pose>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <camera>
          <horizontal_fov>{hfov:.12f}</horizontal_fov>
          <image>
            <width>{CAMERA_WIDTH}</width>
            <height>{CAMERA_HEIGHT}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.02</near>
            <far>10.0</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/qpin_gazebo</namespace>
          </ros>
          <camera_name>camera/color</camera_name>
          <frame_name>camera_color_optical_frame</frame_name>
          <P_cx>{CAMERA_CX}</P_cx>
          <P_cy>{CAMERA_CY}</P_cy>
          <P_fy>{CAMERA_FY}</P_fy>
        </plugin>
      </sensor>"""


def _make_visual_links(urdf_path: Path) -> list[VisualLink]:
    description_dir = urdf_path.parents[1]
    root = ET.parse(urdf_path).getroot()
    links: list[VisualLink] = []
    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        if not link_name:
            continue
        visual_xml_parts = []
        if link_name == "body_link":
            visual_xml_parts.extend(_attached_camera_visuals(root, description_dir))
            camera_sensor = _attached_camera_sensor(root)
            if camera_sensor is not None:
                visual_xml_parts.append(camera_sensor)
        for index, visual in enumerate(link.findall("visual")):
            geometry = _visual_geometry_xml(visual, description_dir)
            if geometry is None:
                continue
            origin = visual.find("origin")
            xyz = _parse_xyz(origin.attrib.get("xyz") if origin is not None else None)
            rpy = _parse_xyz(origin.attrib.get("rpy") if origin is not None else None)
            material = _visual_material_xml(visual)
            visual_xml_parts.append(
                f"""
        <visual name="visual_{index}">
          <pose>{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose>
          {geometry}
          {material}
        </visual>"""
            )
        if not visual_xml_parts:
            continue
        model_name = _safe_model_name(link_name)
        visuals = "\n".join(visual_xml_parts)
        sdf_xml = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <link name="visual_link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      {visuals}
    </link>
  </model>
</sdf>
"""
        links.append(VisualLink(link_name=link_name, model_name=model_name, sdf_xml=sdf_xml))
    return links


def _matrix_from_tf(transform) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    msg = transform.transform
    translation = (msg.translation.x, msg.translation.y, msg.translation.z)
    quaternion = (msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)
    return translation, quaternion


def _pose_from_parts(
    translation: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    pose.position.x = float(translation[0])
    pose.position.y = float(translation[1])
    pose.position.z = float(translation[2])
    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])
    return pose


def _matrix_from_pose_msg(pose: Pose) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, 3] = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
    matrix[:3, :3] = Rotation.from_quat(
        [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]
    ).as_matrix()
    return matrix


def _pose_from_matrix(matrix: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    quat_xyzw = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x = float(quat_xyzw[0])
    pose.orientation.y = float(quat_xyzw[1])
    pose.orientation.z = float(quat_xyzw[2])
    pose.orientation.w = float(quat_xyzw[3])
    return pose


class GazeboRobotVisualFollower(Node):
    def __init__(
        self,
        *,
        visual_links: list[VisualLink],
        root_frame: str,
        world_pose_topic: str,
        ready_topic: str,
        rate_hz: float,
    ) -> None:
        super().__init__("qpin_gazebo_robot_visual_follower")
        self.visual_links = visual_links
        self.root_frame = str(root_frame)
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.state_client = self.create_client(SetEntityState, "/set_entity_state")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world_pose_topic = str(world_pose_topic)
        self.ready_topic = str(ready_topic)
        self.world_from_root = np.eye(4, dtype=float)
        self.period = 1.0 / max(1.0, rate_hz)
        self.timer = None
        self.last_warn_time = 0.0
        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_pub = self.create_publisher(Bool, self.ready_topic, ready_qos)
        self.world_pose_sub = self.create_subscription(
            PoseStamped,
            self.world_pose_topic,
            self._on_world_pose,
            10,
        )
        self.get_logger().info(
            f"following {len(self.visual_links)} visual links from root frame {self.root_frame}; "
            f"world pose topic: {self.world_pose_topic}"
        )

    def _on_world_pose(self, msg: PoseStamped) -> None:
        self.world_from_root = _matrix_from_pose_msg(msg.pose)

    def wait_until_ready(self) -> None:
        while rclpy.ok() and not self.spawn_client.wait_for_service(timeout_sec=0.5):
            pass
        while rclpy.ok() and not self.state_client.wait_for_service(timeout_sec=0.5):
            pass

    def wait_for_initial_tf(self, *, timeout_sec: float = 10.0) -> None:
        deadline = time.monotonic() + float(timeout_sec)
        pending = {visual_link.link_name for visual_link in self.visual_links}
        while rclpy.ok() and pending and time.monotonic() < deadline:
            ready = set()
            for link_name in pending:
                try:
                    self._lookup_link_pose(link_name)
                except TransformException:
                    continue
                ready.add(link_name)
            pending.difference_update(ready)
            if pending:
                rclpy.spin_once(self, timeout_sec=0.1)
        if pending:
            self.get_logger().warn(f"spawning before all TF frames are ready: {sorted(pending)}")

    def _lookup_link_pose(self, link_name: str) -> Pose:
        if link_name == self.root_frame:
            root_from_link = np.eye(4, dtype=float)
        else:
            transform = self.tf_buffer.lookup_transform(
                self.root_frame,
                link_name,
                rclpy.time.Time(),
            )
            root_from_link = _matrix_from_pose_msg(_pose_from_parts(*_matrix_from_tf(transform)))
        world_from_link = self.world_from_root @ root_from_link
        return _pose_from_matrix(world_from_link)

    def spawn_visual_models(self) -> None:
        for visual_link in self.visual_links:
            request = SpawnEntity.Request()
            request.name = visual_link.model_name
            request.xml = visual_link.sdf_xml
            request.robot_namespace = ""
            request.reference_frame = "world"
            try:
                request.initial_pose = self._lookup_link_pose(visual_link.link_name)
            except TransformException:
                request.initial_pose = _pose_from_parts((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done():
                self.get_logger().warn(f"spawn timeout: {visual_link.model_name}")
                continue
            response = future.result()
            if response is not None and not response.success and "already exists" not in response.status_message:
                self.get_logger().warn(f"spawn failed for {visual_link.model_name}: {response.status_message}")
        self.get_logger().info("Gazebo visual models are spawned and following TF")
        self.ready_pub.publish(Bool(data=True))
        self.timer = self.create_timer(self.period, self._publish_link_states)

    def _publish_link_states(self) -> None:
        for visual_link in self.visual_links:
            try:
                pose = self._lookup_link_pose(visual_link.link_name)
            except TransformException as exc:
                now = time.monotonic()
                if now - self.last_warn_time > 2.0:
                    self.last_warn_time = now
                    self.get_logger().warn(f"waiting for {self.root_frame} -> link TF: {exc}")
                return
            state = EntityState()
            state.name = visual_link.model_name
            state.reference_frame = "world"
            state.pose = pose
            request = SetEntityState.Request()
            request.state = state
            self.state_client.call_async(request)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        required=True,
        help="URDF used for simulation visuals.",
    )
    parser.add_argument(
        "--exclude-link",
        action="append",
        default=[],
        help="URDF link visual to skip. Can be passed more than once.",
    )
    parser.add_argument("--root-frame", default="base_link")
    parser.add_argument("--world-pose-topic", default="/qpin_sim/world_from_base")
    parser.add_argument("--ready-topic", default="/qpin_sim/visual_ready")
    parser.add_argument("--rate-hz", type=float, default=15.0)
    args, ros_args = parser.parse_known_args()

    urdf_path = Path(args.urdf).expanduser().resolve()
    excluded_links = set(args.exclude_link)
    visual_links = [
        visual_link
        for visual_link in _make_visual_links(urdf_path)
        if visual_link.link_name not in excluded_links
    ]
    if not visual_links:
        raise RuntimeError("no visual links found in URDF")

    rclpy.init(args=ros_args)
    node = GazeboRobotVisualFollower(
        visual_links=visual_links,
        root_frame=args.root_frame,
        world_pose_topic=args.world_pose_topic,
        ready_topic=args.ready_topic,
        rate_hz=args.rate_hz,
    )
    try:
        node.wait_until_ready()
        node.wait_for_initial_tf()
        node.spawn_visual_models()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
