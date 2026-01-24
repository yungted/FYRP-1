#!/usr/bin/env python3
"""MQTT to ROS2 Bridge - Runs on Jetson (FIXED VERSION)"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json
import threading
import time

MQTT_BROKER = "192.168.0.165"
ROS_CMD_TOPIC = "robot/ros_cmd"
ROS_FEEDBACK_TOPIC = "robot/ros_feedback"
CMD_VEL_TOPIC = "/diffbot_base_controller/cmd_vel_unstamped"

# Velocity command publish rate (Hz)
CMD_VEL_RATE = 10  # 10Hz = publish every 0.1 seconds

class MqttRosBridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros_bridge')
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        
        # Movement control
        self.movement_lock = threading.Lock()
        self.stop_movement_flag = False
        
        self.mqtt_client = mqtt.Client(client_id="jetson-ros-bridge")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(MQTT_BROKER, 1883, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"Connected to MQTT: {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"MQTT connection failed: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connected (rc={rc})")
        client.subscribe(ROS_CMD_TOPIC)
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            command_name = payload.get('command', 'unknown')
            self.get_logger().info(f"Received: {command_name}")
            
            # Send feedback
            self.mqtt_client.publish(ROS_FEEDBACK_TOPIC, json.dumps({
                "type": "command_received", "command": command_name
            }))
            
            # Emergency stop
            if payload.get("emergency") or command_name == "emergency_stop":
                self.stop_robot()
                return
            
            # Extract twist and duration
            twist_data = payload.get("twist", {})
            linear = twist_data.get("linear", {})
            angular = twist_data.get("angular", {})
            duration = float(payload.get("duration", 0))
            
            # ============================================
            # CRITICAL FIX: LOG THE DURATION BEING USED
            # ============================================
            self.get_logger().info(f"Duration from message: {duration} seconds")
            
            linear_x = float(linear.get("x", 0.0))
            angular_z = float(angular.get("z", 0.0))
            
            self.get_logger().info(f"Publishing: linear_x={linear_x}, angular_z={angular_z}")
            
            # ============================================
            # CRITICAL FIX: CONTINUOUSLY PUBLISH VELOCITY
            # Stop any ongoing movement first
            # ============================================
            self.stop_movement_flag = True
            time.sleep(0.1)  # Let previous movement stop
            
            # Execute movement with continuous publishing
            if duration > 0:
                threading.Thread(
                    target=self.execute_movement,
                    args=(linear_x, angular_z, duration, command_name),
                    daemon=True
                ).start()
            else:
                # Duration 0 = stop command
                self.stop_robot()
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
    
    def execute_movement(self, linear_x, angular_z, duration, command_name):
        """
        CRITICAL FIX: Continuously publish velocity commands at 10Hz
        during the entire duration, then stop.
        """
        self.stop_movement_flag = False
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        
        start_time = time.time()
        rate = 1.0 / CMD_VEL_RATE  # 0.1 seconds for 10Hz
        
        self.get_logger().info(f"Starting movement for {duration} seconds")
        
        # Continuously publish velocity during the duration
        while (time.time() - start_time) < duration and not self.stop_movement_flag:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(rate)
        
        # Stop the robot
        self.stop_robot()
        
        actual_duration = time.time() - start_time
        self.get_logger().info(f"Movement completed after {actual_duration:.2f} seconds")
        
        # Send completion feedback
        self.mqtt_client.publish(ROS_FEEDBACK_TOPIC, json.dumps({
            "type": "command_completed",
            "command": command_name,
            "duration": actual_duration
        }))
    
    def stop_robot(self):
        """Immediately stop the robot."""
        self.stop_movement_flag = True
        stop_twist = Twist()  # All zeros
        # Publish multiple times to ensure it's received
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.05)
        self.get_logger().info("Robot stopped")

def main():
    rclpy.init()
    node = MqttRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.mqtt_client.loop_stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
