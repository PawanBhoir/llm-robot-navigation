#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import urllib.request
import json
import re
import yaml
import os
import threading
import math
from ament_index_python.packages import get_package_share_directory

# TODO 1: System prompt structured framework definition
_SYSTEM = """You are an absolute JSON translation engine for robot navigation commands.
You must output exclusively valid JSON matching one of the schemas below. Do not include any introductory prose, summary markdown blocks, or conversational explanations.

Available named locations coordinates index profiles:
{locations}

Current runtime user statement input text: "{command}"

Permitted Output Schemas:
1. Destination Goal Found: {{"action": "go", "target": "location_name"}} or {{"action": "go", "x": 2.5, "y": 1.0}}
2. Emergency Hold Request: {{"action": "stop"}}
3. Unrecognized instruction fallback option: {{"action": "unknown"}}

Examples:
Input: "Please quickly drive over to room_a" -> Output: {{"action": "go", "target": "room_a"}}
Input: "Halt immediately!" -> Output: {{"action": "stop"}}
Input: "Navigate to coordinates 1.2 -3.4 please" -> Output: {{"action": "go", "x": 1.2, "y": -3.4}}
"""

class LLMNavigator(Node):
    def __init__(self):
        super().__init__('llm_navigator')
        
        self.cmd_sub = self.create_subscription(String, '/llm_command', self.command_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Load local named spatial coordinates database
        pkg_dir = get_package_share_directory('diff_drive_robot')
        loc_path = os.path.join(pkg_dir, 'config', 'locations.yaml')
        
        if os.path.exists(loc_path):
            with open(loc_path, 'r') as f:
                self._locations = yaml.safe_load(f).get('locations', {})
        else:
            self.get_logger().warn("locations.yaml configuration profile missing.")
            self._locations = {}

        self.ollama_url = "http://localhost:11434"
        self._busy = False
        self._goal_xy = (0.0, 0.0)
        self.get_logger().info("LLM Executive Navigation Layer successfully deployed.")

    def command_callback(self, msg):
        if self._busy:
            self.get_logger().warn("System busy executing prior instruction payload.")
            return
        self._busy = True
        threading.Thread(target=self.process_command_thread, args=(msg.data,), daemon=True).start()

    # TODO 2: Ollama API call execution logic loop via stdlib
    def call_ollama(self, prompt):
        url = f"{self.ollama_url}/api/generate"
        body = {
            "model": "tinyllama",
            "prompt": prompt,
            "stream": False,
            "format": "json"
        }
        try:
            req = urllib.request.Request(
                url, 
                data=json.dumps(body).encode('utf-8'),
                headers={'Content-Type': 'application/json'}
            )
            with urllib.request.urlopen(req, timeout=30) as res:
                resp_bytes = res.read()
                resp_json = json.loads(resp_bytes.decode('utf-8'))
                return resp_json.get("response", "")
        except Exception as e:
            self.get_logger().error(f"Failed to query Ollama service engine backend: {e}")
            return ""

    # TODO 3: Regex JSON target string parser extraction
    def _extract_json(self, raw_text):
        match = re.search(r'\{.*?\}', raw_text, re.DOTALL)
        if match:
            try:
                return json.loads(match.group(0))
            except json.JSONDecodeError:
                return None
        return None

    # TODO 4: Transform prompt outcomes to actionable coordinate profiles
    def _resolve_goal(self, data):
        action = data.get("action")
        if action == "stop":
            self.get_logger().info("Executing active path cancellation request.")
            self.nav_client.prune_goals()
            return None
        elif action == "unknown":
            self.get_logger().warn("LLM prompt parsing could not determine target intent.")
            return None
        elif action == "go":
            if "target" in data:
                name = data["target"]
                if name in self._locations:
                    loc = self._locations[name]
                    return loc['x'], loc['y'], loc.get('yaw', 0.0)
                else:
                    self.get_logger().error(f"Hallucinated location requested: '{name}' not found in configuration indices.")
                    return None
            elif "x" in data and "y" in data:
                return float(data["x"]), float(data["y"]), 0.0
        return None

    def process_command_thread(self, user_command):
        loc_summary = ", ".join(list(self._locations.keys()))
        full_prompt = _SYSTEM.format(locations=loc_summary, command=user_command)
        
        raw_response = self.call_ollama(full_prompt)
        parsed_json = self._extract_json(raw_response)
        
        if parsed_json:
            goal = self._resolve_goal(parsed_json)
            if goal:
                self._send_goal(goal[0], goal[1], goal[2])
                return
        self._busy = False

    # TODO 5: Asynchronous non-blocking goal dispatch background routines
    def _send_goal(self, x, y, yaw_deg):
        threading.Thread(target=self._send_goal_thread, args=(x, y, yaw_deg), daemon=True).start()

    def _send_goal_thread(self, x, y, yaw_deg):
        self.nav_client.wait_for_server(timeout_sec=10.0)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        
        # Convert degrees yaw orientation metrics to Quaternion parameters
        rad = math.radians(yaw_deg)
        goal_msg.pose.orientation.z = math.sin(rad / 2.0)
        goal_msg.pose.orientation.w = math.cos(rad / 2.0)
        
        self._goal_xy = (x, y)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info(f"Dispatching verified goal vector to Nav2: x={x}, y={y}")
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 server rejected the dispatched goal profile.")
            self._busy = False
            return
        res_future = handle.get_result_async()
        res_future.add_done_callback(self._result_cb)

    # TODO 6: Dynamic metric feedback collection handling evaluations
    def _result_cb(self, future):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.get_logger().info(f"Target navigation complete. Mission execution tracking elapsed: {elapsed:.2f}s")
        self._busy = False

def main(args=None):
    rclpy.init(args=args)
    node = LLMNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()