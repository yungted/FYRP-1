#!/usr/bin/env python3
"""
Enhanced MQTT to ROS2 Bridge for Jetson Orin Nano
===================================================

Features:
- Handles both low-level movement commands (velocity control)
- High-level navigation requests using Nav2's NavigateToPose action
- Supports emergency stop with safety limits
- Publishes feedback to PC about command execution and navigation status
- Graceful handling of errors and timeouts
- Context-aware navigation with pose estimation

ROS Topics:
- /diffbot_base_controller/cmd_vel_unstamped: Direct velocity commands
- /navigate_to_pose: Nav2 action for autonomous navigation
- /amcl_pose: Current robot pose (with covariance)

MQTT Topics:
- robot/ros_cmd: Receive movement and navigation commands
- robot/ros_feedback: Send feedback about command execution
- robot/waypoint_feedback: Send waypoint-specific navigation feedback
- robot/current_pose: Publish current robot position

Safety Features:
- Velocity limiting to prevent dangerous movements
- Emergency stop that immediately halts all motion
- Nav2 timeout handling
- Continuous velocity publishing during movements
- Automatic stop after command completes
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import paho.mqtt.client as mqtt
import json
import threading
import time
import math
from typing import Optional, Dict, Tuple

# =========================================================
# MQTT CONFIGURATION
# =========================================================
MQTT_BROKER = "192.168.0.165"
MQTT_PORT = 1883
MQTT_TIMEOUT = 60

# Topic Subscriptions
ROS_CMD_TOPIC = "robot/ros_cmd"          # Receive movement commands
WAYPOINT_NAV_TOPIC = "robot/navigate_waypoint"  # Receive waypoint navigation requests

# Topic Publications
ROS_FEEDBACK_TOPIC = "robot/ros_feedback"        # Send command execution feedback
WAYPOINT_FEEDBACK_TOPIC = "robot/waypoint_feedback"  # Send navigation feedback
CURRENT_POSE_TOPIC = "robot/current_pose"        # Publish current pose

# =========================================================
# ROS CONFIGURATION
# =========================================================
CMD_VEL_TOPIC = "/diffbot_base_controller/cmd_vel_unstamped"  # Velocity command topic
AMCL_POSE_TOPIC = "/amcl_pose"                                 # Robot's current pose
INIT_POSE_TOPIC = "/initialpose"                               # Initial pose for AMCL

# Velocity command publish rate for continuous movement
CMD_VEL_PUBLISH_RATE = 10  # Hz (publish every 0.1 seconds)
CMD_VEL_RATE_PERIOD = 1.0 / CMD_VEL_PUBLISH_RATE

# =========================================================
# SAFETY LIMITS
# =========================================================
MAX_LINEAR_VELOCITY = 0.5    # m/s - Maximum forward/backward speed
MAX_ANGULAR_VELOCITY = 0.8   # rad/s - Maximum rotation speed
NAV_TIMEOUT_SECONDS = 300    # 5 minutes - timeout for autonomous navigation
MOVEMENT_COMMAND_TIMEOUT = 15  # seconds - timeout for velocity commands


class MqttRosBridge(Node):
    """
    Bridge between MQTT commands (from PC) and ROS2 (on Jetson).
    Handles both low-level movement and high-level navigation.
    """
    
    def __init__(self):
        super().__init__('mqtt_ros_bridge')
        
        # ====== ROS Publishers/Subscribers/Action Clients ======
        
        # Velocity command publisher (for low-level movement)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10
        )
        
        # Initial pose publisher (for AMCL pose estimation)
        initial_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            INIT_POSE_TOPIC,
            initial_pose_qos
        )
        
        # Current pose subscriber (from AMCL)
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            AMCL_POSE_TOPIC,
            self.on_amcl_pose,
            amcl_qos
        )
        
        # Odometry subscriber (for feedback)
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.on_odom,
            10
        )
        
        # Navigation action client (for Nav2)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )
        
        # ====== MQTT Setup ======
        self.mqtt_client = mqtt.Client(client_id="jetson-ros-bridge")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, MQTT_TIMEOUT)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"✅ MQTT Connected: {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Failed: {e}")
        
        # ====== State Management ======
        self.movement_lock = threading.Lock()
        self.stop_movement_flag = False
        self.active_nav_goal_handle = None  # Track active navigation goal
        
        # Current pose tracking
        self.current_pose = {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "quaternion": {"z": 0.0, "w": 1.0}
        }
        
        self.get_logger().info("🤖 MQTT-ROS Bridge initialized successfully")
    
    # =====================================================================
    # MQTT CALLBACKS
    # =====================================================================
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Called when MQTT connection is established."""
        self.get_logger().info(f"📡 MQTT connected (rc={rc})")
        # Subscribe to command topics
        client.subscribe(ROS_CMD_TOPIC)
        client.subscribe(WAYPOINT_NAV_TOPIC)
        self.get_logger().info(f"   Subscribed to: {ROS_CMD_TOPIC}, {WAYPOINT_NAV_TOPIC}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """
        Route MQTT messages to appropriate handlers.
        
        Message types:
        1. robot/ros_cmd: Low-level movement commands {twist, duration}
        2. robot/navigate_waypoint: High-level navigation {waypoint, pose}
        """
        try:
            payload = json.loads(msg.payload.decode())
            self.get_logger().info(f"📨 Received on {msg.topic}")
            
            if msg.topic == ROS_CMD_TOPIC:
                # Low-level movement command
                self._handle_ros_command(payload)
            
            elif msg.topic == WAYPOINT_NAV_TOPIC:
                # High-level waypoint navigation
                self._handle_waypoint_navigation(payload)
            
            else:
                self.get_logger().warn(f"⚠️ Unknown topic: {msg.topic}")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ JSON Decode Error: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Message Handler Error: {e}")
    
    # =====================================================================
    # ROS CALLBACKS
    # =====================================================================
    
    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """
        Callback for AMCL pose updates.
        Tracks robot's current position and publishes to MQTT for PC context.
        """
        try:
            # Extract position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Extract quaternion
            quat = msg.pose.pose.orientation
            z = quat.z
            w = quat.w
            
            # Calculate yaw (theta) from quaternion
            # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
            theta = math.atan2(2 * (w * z), 1 - 2 * (z * z))
            theta_deg = math.degrees(theta)
            
            # Store pose
            self.current_pose = {
                "x": round(x, 4),
                "y": round(y, 4),
                "theta": round(theta_deg, 2),
                "quaternion": {"z": round(z, 4), "w": round(w, 4)}
            }
            
            # Publish current pose to PC for context awareness
            pose_msg = {
                "x": self.current_pose["x"],
                "y": self.current_pose["y"],
                "z": self.current_pose["quaternion"]["z"],
                "w": self.current_pose["quaternion"]["w"],
                "theta": self.current_pose["theta"]
            }
            self.mqtt_client.publish(CURRENT_POSE_TOPIC, json.dumps(pose_msg))
            
            self.get_logger().debug(
                f"📍 Pose: x={x:.2f}, y={y:.2f}, θ={theta_deg:.1f}°"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing AMCL pose: {e}")
    
    def on_odom(self, msg: Odometry):
        """
        Callback for odometry updates.
        Used for cross-checking position and detecting movement errors.
        """
        try:
            # This can be used to detect drift or errors
            # For now, we'll just acknowledge it exists
            pass
        except Exception as e:
            self.get_logger().error(f"❌ Odometry error: {e}")
    
    # =====================================================================
    # COMMAND HANDLERS
    # =====================================================================
    
    def _handle_ros_command(self, payload: dict):
        """
        Handle low-level ROS movement commands.
        
        Expected message format:
        {
            "command": "move_forward",
            "twist": {
                "linear": {"x": 0.3, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "duration": 2.0,
            "expected_distance_m": 0.6,
            "expected_rotation_deg": 0.0,
            "emergency": false
        }
        """
        command_name = payload.get('command', 'unknown')
        self.get_logger().info(f"🎮 Command: {command_name}")
        
        try:
            # Send acknowledgment to PC
            self.mqtt_client.publish(ROS_FEEDBACK_TOPIC, json.dumps({
                "type": "command_received",
                "command": command_name,
                "timestamp": time.time()
            }))
            
            # Handle emergency stop
            if payload.get("emergency") or command_name == "emergency_stop":
                self._stop_robot()
                self.mqtt_client.publish(ROS_FEEDBACK_TOPIC, json.dumps({
                    "type": "command_completed",
                    "command": "emergency_stop",
                    "duration": 0.0
                }))
                return
            
            # Extract movement parameters
            twist_data = payload.get("twist", {})
            linear = twist_data.get("linear", {})
            angular = twist_data.get("angular", {})
            duration = float(payload.get("duration", 0.0))
            
            # Validate and clamp velocities
            linear_x = self._clamp_velocity(
                float(linear.get("x", 0.0)),
                MAX_LINEAR_VELOCITY
            )
            angular_z = self._clamp_velocity(
                float(angular.get("z", 0.0)),
                MAX_ANGULAR_VELOCITY
            )
            
            self.get_logger().info(
                f"📋 Parameters: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}, "
                f"duration={duration:.2f}s"
            )
            
            # Execute movement in thread
            if duration > 0:
                threading.Thread(
                    target=self._execute_movement,
                    args=(linear_x, angular_z, duration, command_name),
                    daemon=True
                ).start()
            else:
                # Duration 0 = stop command
                self._stop_robot()
                self.mqtt_client.publish(ROS_FEEDBACK_TOPIC, json.dumps({
                    "type": "command_completed",
                    "command": command_name,
                    "duration": 0.0
                }))
                
        except Exception as e:
            self.get_logger().error(f"❌ Command execution error: {e}")
            self.mqtt_client.publish(ROS_FEEDBACK_TOPIC, json.dumps({
                "type": "error",
                "message": str(e)
            }))
    
    def _handle_waypoint_navigation(self, payload: dict):
        """
        Handle high-level waypoint navigation via Nav2.
        
        Expected message format:
        {
            "action": "navigate_to_pose",
            "waypoint": "kitchen",
            "pose": {
                "x": 1.97,
                "y": 12.24,
                "z": 0.0,
                "w": 1.0
            },
            "description": "Main kitchen area"
        }
        """
        waypoint = payload.get("waypoint", "unknown")
        description = payload.get("description", waypoint)
        
        self.get_logger().info(f"🗺️ Navigation Request: {waypoint} ({description})")
        
        try:
            # Send acknowledgment
            self.mqtt_client.publish(WAYPOINT_FEEDBACK_TOPIC, json.dumps({
                "type": "navigation_started",
                "waypoint": waypoint,
                "timestamp": time.time()
            }))
            
            # Extract pose
            pose_data = payload.get("pose", {})
            x = float(pose_data.get("x", 0.0))
            y = float(pose_data.get("y", 0.0))
            z = float(pose_data.get("z", 0.0))
            w = float(pose_data.get("w", 1.0))
            
            # Normalize quaternion
            q_norm = math.sqrt(z*z + w*w)
            if q_norm > 0:
                z /= q_norm
                w /= q_norm
            
            # Start navigation in thread
            threading.Thread(
                target=self._navigate_to_pose,
                args=(x, y, z, w, waypoint),
                daemon=True
            ).start()
            
        except Exception as e:
            self.get_logger().error(f"❌ Navigation error: {e}")
            self.mqtt_client.publish(WAYPOINT_FEEDBACK_TOPIC, json.dumps({
                "type": "navigation_failed",
                "waypoint": waypoint,
                "reason": str(e)
            }))
    
    # =====================================================================
    # MOVEMENT EXECUTION
    # =====================================================================
    
    def _execute_movement(self, linear_x: float, angular_z: float, duration: float, 
                         command_name: str):
        """
        Execute a velocity-controlled movement for a specified duration.
        
        Continuously publishes twist messages at CMD_VEL_PUBLISH_RATE to ensure
        smooth robot motion. Stops after duration expires.
        
        Args:
            linear_x: Forward velocity (m/s)
            angular_z: Rotation rate (rad/s)
            duration: How long to execute (seconds)
            command_name: Name of command for feedback
        """
        with self.movement_lock:
            self.stop_movement_flag = False
            
            # Create twist message
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = angular_z
            
            # Calculate expected distance/angle for logging
            expected_distance = abs(linear_x) * duration
            expected_angle_rad = abs(angular_z) * duration
            expected_angle_deg = math.degrees(expected_angle_rad)
            
            start_time = time.time()
            publish_count = 0
            
            self.get_logger().info(
                f"▶️ Starting movement: {command_name} for {duration:.2f}s "
                f"(expect ~{expected_distance:.2f}m or {expected_angle_deg:.1f}°)"
            )
            
            # Continuously publish velocity during duration
            while (time.time() - start_time) < duration and not self.stop_movement_flag:
                self.cmd_vel_pub.publish(twist_msg)
                publish_count += 1
                time.sleep(CMD_VEL_RATE_PERIOD)
            
            # Stop the robot
            self._stop_robot()
            
            actual_duration = time.time() - start_time
            self.get_logger().info(
                f"⏹️ Movement complete: {command_name} after {actual_duration:.2f}s "
                f"({publish_count} velocity updates)"
            )
            
            # Send completion feedback
            self.mqtt_client.publish(ROS_FEEDBACK_TOPIC, json.dumps({
                "type": "command_completed",
                "command": command_name,
                "duration": round(actual_duration, 2),
                "publish_count": publish_count
            }))
    
    def _navigate_to_pose(self, x: float, y: float, z: float, w: float, 
                         waypoint: str):
        """
        Autonomously navigate to a target pose using Nav2.
        
        Uses the NavigateToPose action to handle obstacle avoidance,
        path planning, and autonomous navigation.
        
        Args:
            x, y: Target position in map frame
            z, w: Target orientation as quaternion
            waypoint: Waypoint name for feedback
        """
        try:
            # Wait for action server
            self.get_logger().info(f"⏳ Waiting for Nav2 action server...")
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError("Nav2 action server not available")
            
            self.get_logger().info(f"✅ Nav2 server ready")
            
            # Create navigation goal
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.position.z = 0.0  # Always 0 for 2D navigation
            
            goal.pose.pose.orientation.x = 0.0
            goal.pose.pose.orientation.y = 0.0
            goal.pose.pose.orientation.z = z
            goal.pose.pose.orientation.w = w
            
            self.get_logger().info(
                f"🎯 Sending goal to Nav2: x={x:.2f}, y={y:.2f}, waypoint='{waypoint}'"
            )
            
            # Send goal async
            send_goal_future = self.nav_client.send_goal_async(goal)
            
            # Wait for goal to be accepted (with timeout)
            import rclpy.time
            start = time.time()
            while not send_goal_future.done():
                if (time.time() - start) > 10.0:
                    raise RuntimeError("Timeout waiting for goal acceptance")
                time.sleep(0.1)
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                raise RuntimeError("Navigation goal was rejected by Nav2")
            
            self.active_nav_goal_handle = goal_handle
            self.get_logger().info(f"✅ Navigation goal accepted")
            
            # Wait for result (with timeout)
            result_future = goal_handle.get_result_async()
            start = time.time()
            
            while not result_future.done():
                elapsed = time.time() - start
                if elapsed > NAV_TIMEOUT_SECONDS:
                    self.get_logger().warn(f"⏱️ Navigation timeout ({elapsed:.0f}s)")
                    goal_handle.cancel_goal_async()
                    break
                time.sleep(0.5)
            
            if result_future.done():
                result = result_future.result()
                self.get_logger().info(f"🏁 Navigation result: {result}")
                
                # Send success feedback
                self.mqtt_client.publish(WAYPOINT_FEEDBACK_TOPIC, json.dumps({
                    "type": "waypoint_reached",
                    "waypoint": waypoint,
                    "final_pose": self.current_pose,
                    "timestamp": time.time()
                }))
            
        except Exception as e:
            self.get_logger().error(f"❌ Navigation failed: {e}")
            self.mqtt_client.publish(WAYPOINT_FEEDBACK_TOPIC, json.dumps({
                "type": "navigation_failed",
                "waypoint": waypoint,
                "reason": str(e)
            }))
        finally:
            self.active_nav_goal_handle = None
    
    # =====================================================================
    # SAFETY UTILITIES
    # =====================================================================
    
    def _stop_robot(self):
        """
        Immediately stop the robot by publishing zero velocities.
        Called after movement completes or on error/emergency.
        """
        self.stop_movement_flag = True
        stop_twist = Twist()  # All zeros
        
        # Publish multiple times to ensure it's received
        for i in range(3):
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.05)
        
        self.get_logger().info("🛑 Robot stopped")
    
    def _clamp_velocity(self, velocity: float, max_velocity: float) -> float:
        """
        Clamp velocity to safe limits.
        
        Args:
            velocity: Requested velocity value
            max_velocity: Maximum allowed absolute value
            
        Returns:
            Clamped velocity within [-max_velocity, max_velocity]
        """
        clamped = max(-max_velocity, min(max_velocity, velocity))
        if clamped != velocity:
            self.get_logger().warn(
                f"⚠️ Velocity clamped: {velocity} -> {clamped} (max={max_velocity})"
            )
        return clamped


def main():
    """Initialize and run the bridge node."""
    rclpy.init()
    node = MqttRosBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down...")
    finally:
        # Cleanup
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
