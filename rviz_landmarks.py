malharlabade@alfrid3:~$ cat /home/malharlabade/rviz_landmarks.py
import json
import math
import os
import subprocess
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

DB_PATH = "/home/malharlabade/waypoints.json"
CORRIDOR_CHAIN = ["midpoint 1", "robot stuff"]
DOORS = ("door 1", "door 2")

def load_waypoints():
    if os.path.exists(DB_PATH):
        with open(DB_PATH, "r") as f:
            return json.load(f)
    return {}

def save_waypoints(data):
    with open(DB_PATH, "w") as f:
        json.dump(data, f, indent=2)

def ask_name_popup(x: float, y: float) -> str:
    try:
        env = os.environ.copy()
        env.setdefault("DISPLAY", ":0")
        env.setdefault("XAUTHORITY", "/home/malharlabade/.Xauthority")
        res = subprocess.run(
            [
                "zenity", "--entry",
                "--title=Save RViz2 Map Point",
                f"--text=Clicked ({x:.2f}, {y:.2f}).\nEnter location name (e.g. desk, kitchen):"
            ],
            capture_output=True, text=True, env=env
        )
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip().lower()
    except Exception:
        pass
    return input(f"\n👉 Clicked ({x:.2f}, {y:.2f})! Type location name here and press Enter: ").strip().lower()

def run_so101_arm_task(location_name: str):
    """Standalone Python hook for your Hiwonder HX-20L SO-ARM101."""
    print(f"[ARM] Arrived at '{location_name}'. Running standalone SO-101 sequence...")

class RVizLandmarks(Node):
    def __init__(self):
        super().__init__('rviz_landmarks')
        self.marker_pub = self.create_publisher(MarkerArray, '/landmark_markers', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.publish_all_markers)

    def get_robot_xy(self):
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.1))
                return (t.transform.translation.x, t.transform.translation.y)
            except Exception:
                continue
        return None

    def find_closest_node(self, waypoints, chain):
        pos = self.get_robot_xy()
        if pos is None:
            print("[WARN] Could not read TF map -> base_footprint!")
            return None, 999.0
        rx, ry = pos
        candidates = []
        for name in list(DOORS) + chain:
            if name in waypoints:
                d = math.hypot(rx - float(waypoints[name]["x"]), ry - float(waypoints[name]["y"]))
                candidates.append((d, name))
        if not candidates:
            return None, 999.0
        dist, best = min(candidates, key=lambda item: item)
        print(f"[POSE] Robot at ({rx:.2f}, {ry:.2f}) — closest landmark: '{best}' ({dist:.2f}m away)")
        return best, dist

    def build_route(self, target: str, waypoints: dict) -> list:
        chain = [step for step in CORRIDOR_CHAIN if step in waypoints]
        if target not in chain and target not in DOORS:
            return [target]

        closest_node, dist = self.find_closest_node(waypoints, chain)
        if closest_node is None or not chain:
            return [target]

        if closest_node == target and dist < 0.35:
            return [target]

        pos = self.get_robot_xy()
        if pos is not None:
            direct_dist = math.hypot(pos[0] - float(waypoints[target]["x"]), pos[1] - float(waypoints[target]["y"]))
            if direct_dist <= 1.25:
                return [target]

        start_idx = 0 if closest_node in DOORS else chain.index(closest_node)
        end_idx = 0 if target in DOORS else chain.index(target)

        route = []
        if start_idx < end_idx:
            for i in range(start_idx, end_idx + 1):
                if i == start_idx and closest_node == chain[i] and dist < 0.35:
                    continue
                route.append(chain[i])
        elif start_idx > end_idx:
            for i in range(start_idx, end_idx - 1, -1):
                if i == start_idx and closest_node == chain[i] and dist < 0.35:
                    continue
                route.append(chain[i])
        else:
            if target in DOORS and closest_node in DOORS and closest_node != target:
                route.append(chain[0])

        if not route or route[-1] != target:
            route.append(target)
        return route

    def publish_all_markers(self):
        waypoints = load_waypoints()
        arr = MarkerArray()
        for idx, (name, pt) in enumerate(waypoints.items()):
            pin = Marker()
            pin.header.frame_id = "map"
            pin.header.stamp = self.get_clock().now().to_msg()
            pin.ns = "landmark_pins"
            pin.id = idx * 2
            pin.type = Marker.CYLINDER
            pin.action = Marker.ADD
            pin.pose.position.x = float(pt["x"])
            pin.pose.position.y = float(pt["y"])
            pin.pose.position.z = 0.05
            pin.pose.orientation.w = 1.0
            pin.scale.x = 0.22
            pin.scale.y = 0.22
            pin.scale.z = 0.10
            pin.color.r = 0.1
            pin.color.g = 0.9
            pin.color.b = 0.2
            pin.color.a = 0.9
            arr.markers.append(pin)

            txt = Marker()
            txt.header.frame_id = "map"
            txt.header.stamp = pin.header.stamp
            txt.ns = "landmark_labels"
            txt.id = idx * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(pt["x"])
            txt.pose.position.y = float(pt["y"])
            txt.pose.position.z = 0.35
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.22
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = name.upper()
            arr.markers.append(txt)

        self.marker_pub.publish(arr)

    def listen_for_clicks(self):
        self.create_subscription(PointStamped, '/clicked_point', self._click_cb, 10)
        print("Listening for RViz2 'Publish Point' clicks... Click any spot on the map now!")
        rclpy.spin(self)

    def _click_cb(self, msg: PointStamped):
        x, y = round(msg.point.x, 3), round(msg.point.y, 3)
        print(f"📍 Detected RViz2 click at ({x}, {y})!")
        name = ask_name_popup(x, y)
        if name:
            waypoints = load_waypoints()
            waypoints[name] = {"x": x, "y": y}
            save_waypoints(waypoints)
            self.publish_all_markers()
            print(f"✅ Saved '{name}' to {DB_PATH}!")

    def _drive_single_point(self, label: str, pt: dict) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pt["x"])
        goal.pose.pose.position.y = float(pt["y"])
        goal.pose.pose.orientation.w = 1.0

        print(f"[BASE] Driving to '{label}' ({pt['x']}, {pt['y']})...")
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            print(f"[BASE] Goal '{label}' rejected.")
            return False

        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        status = res_future.result().status
        if status == 4:
            print(f"[BASE] Reached '{label}'!")
            return True
        print(f"[BASE] Stopped before reaching '{label}' (status {status}).")
        return False

    def go_to(self, label: str):
        waypoints = load_waypoints()
        norm = label.replace(" ", "")
        matched = next((k for k in waypoints if k.replace(" ", "") == norm), None)
        if matched is None:
            print(f"Location '{label}' not found. Saved: {list(waypoints.keys())}")
            return
        label = matched

        self.publish_all_markers()
        self.nav_client.wait_for_server()

        route = self.build_route(label, waypoints)
        print(f"[ROUTE] Path to '{label}': {' -> '.join(route)}")

        for step in route:
            if not self._drive_single_point(step, waypoints[step]):
                return

        run_so101_arm_task(label)

def main():
    rclpy.init()
    node = RVizLandmarks()
    try:
        if len(sys.argv) >= 3 and sys.argv[1].lower() == "go":
            node.go_to(" ".join(sys.argv[2:]).lower())
        else:
            node.listen_for_clicks()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
malharlabade@alfrid3:~$ 
