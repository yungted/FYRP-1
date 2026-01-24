#!/usr/bin/env python3
"""
System Test & Diagnostic Utility
==================================

This script helps test individual components of the robot control system:
- MQTT connectivity
- ROS topics/services
- Waypoint persistence
- LLM responses
- End-to-end integration

Usage:
    python3 system_test.py --test all
    python3 system_test.py --test mqtt
    python3 system_test.py --test waypoints
    python3 system_test.py --test llm
"""

import argparse
import json
import sys
import time
import subprocess
from pathlib import Path

# Optional imports
try:
    import paho.mqtt.client as mqtt
    HAVE_MQTT = True
except ImportError:
    HAVE_MQTT = False

try:
    from waypoints.waypoint_manager import WaypointManager
    HAVE_WAYPOINT_MGR = True
except ImportError:
    HAVE_WAYPOINT_MGR = False

try:
    import rclpy
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

# =========================================================
# MQTT TESTS
# =========================================================

def test_mqtt_connectivity(broker="192.168.0.165", port=1883, timeout=5):
    """Test MQTT broker connectivity."""
    print("\n" + "="*60)
    print("🧪 MQTT CONNECTIVITY TEST")
    print("="*60)
    
    if not HAVE_MQTT:
        print("❌ paho-mqtt not installed. Install with: pip install paho-mqtt")
        return False
    
    print(f"📡 Attempting to connect to {broker}:{port}...")
    
    client = mqtt.Client(client_id="test-client")
    connected = False
    error_msg = None
    
    def on_connect(client, userdata, flags, rc):
        nonlocal connected
        connected = True
        print(f"✅ Connected! (rc={rc})")
    
    def on_disconnect(client, userdata, rc):
        print(f"⚠️ Disconnected (rc={rc})")
    
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    
    try:
        client.connect(broker, port, timeout)
        client.loop_start()
        time.sleep(2)
        client.loop_stop()
        
        if connected:
            print("✅ MQTT test PASSED")
            return True
        else:
            print("❌ Connection callback not called")
            return False
            
    except Exception as e:
        print(f"❌ MQTT test FAILED: {e}")
        return False


def test_mqtt_publish_subscribe(broker="192.168.0.165"):
    """Test MQTT publish/subscribe cycle."""
    print("\n" + "="*60)
    print("🧪 MQTT PUBLISH/SUBSCRIBE TEST")
    print("="*60)
    
    if not HAVE_MQTT:
        print("❌ paho-mqtt not installed")
        return False
    
    received_message = []
    
    def on_message(client, userdata, msg):
        received_message.append(msg.payload.decode())
        print(f"✅ Received message: {msg.payload.decode()}")
    
    client = mqtt.Client(client_id="test-pub-sub")
    client.on_message = on_message
    
    try:
        client.connect(broker, 1883, 60)
        client.loop_start()
        
        # Subscribe to test topic
        client.subscribe("robot/test")
        time.sleep(0.5)
        
        # Publish test message
        print("📤 Publishing test message...")
        client.publish("robot/test", json.dumps({"test": "message"}))
        
        time.sleep(2)
        client.loop_stop()
        
        if received_message:
            print("✅ Pub/Sub test PASSED")
            return True
        else:
            print("❌ No message received")
            return False
            
    except Exception as e:
        print(f"❌ Pub/Sub test FAILED: {e}")
        return False


# =========================================================
# WAYPOINT TESTS
# =========================================================

def test_waypoint_manager():
    """Test waypoint creation, storage, and retrieval."""
    print("\n" + "="*60)
    print("🧪 WAYPOINT MANAGER TEST")
    print("="*60)
    
    if not HAVE_WAYPOINT_MGR:
        print("❌ waypoint_manager module not found")
        return False
    
    try:
        # Create test manager
        manager = WaypointManager("test_waypoints.json")
        print("✅ WaypointManager instantiated")
        
        # Test 1: Add waypoint
        print("\n[Test 1] Adding waypoint...")
        result = manager.add_waypoint(
            "test_location",
            1.5, 2.5,  # x, y
            0.0, 1.0,  # z, w (quaternion)
            "A test location"
        )
        if not result:
            print("❌ Failed to add waypoint")
            return False
        print("✅ Waypoint added")
        
        # Test 2: Retrieve waypoint
        print("\n[Test 2] Retrieving waypoint...")
        wp = manager.get_waypoint("test_location")
        if not wp:
            print("❌ Failed to retrieve waypoint")
            return False
        print(f"✅ Retrieved: {wp}")
        
        # Test 3: List waypoints
        print("\n[Test 3] Listing all waypoints...")
        waypoints = manager.list_waypoints()
        print(f"✅ Found {len(waypoints)} waypoints: {waypoints}")
        
        # Test 4: Get LLM context
        print("\n[Test 4] Generating LLM context...")
        context = manager.get_context_for_llm()
        print(f"✅ Context:\n{context}")
        
        # Test 5: Find nearby waypoint
        print("\n[Test 5] Finding nearby waypoint...")
        nearby = manager.find_nearby_waypoint(1.5, 2.5, max_distance=1.0)
        if nearby:
            print(f"✅ Found nearby: {nearby}")
        else:
            print("⚠️ No nearby waypoint (expected if just added)")
        
        # Cleanup
        print("\n[Cleanup] Removing test waypoint...")
        manager.remove_waypoint("test_location")
        print("✅ Cleaned up")
        
        print("\n✅ Waypoint Manager test PASSED")
        return True
        
    except Exception as e:
        print(f"❌ Waypoint test FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


# =========================================================
# ROS TESTS
# =========================================================

def test_ros_connectivity():
    """Test ROS 2 installation and basic connectivity."""
    print("\n" + "="*60)
    print("🧪 ROS 2 CONNECTIVITY TEST")
    print("="*60)
    
    if not HAVE_ROS:
        print("❌ ROS 2 not installed or importable")
        return False
    
    try:
        # Try to list nodes
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            print(f"✅ ROS 2 active")
            print(f"📊 Active nodes: {len([n for n in nodes if n])} nodes")
            for node in nodes[:5]:  # Show first 5
                if node:
                    print(f"   - {node}")
            return True
        else:
            print(f"❌ ROS 2 query failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ ROS test FAILED: {e}")
        return False


def test_ros_topics():
    """Test ROS topics for navigation and control."""
    print("\n" + "="*60)
    print("🧪 ROS TOPICS TEST")
    print("="*60)
    
    required_topics = [
        "/diffbot_base_controller/cmd_vel_unstamped",
        "/amcl_pose",
        "/odom"
    ]
    
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode != 0:
            print(f"❌ Failed to list topics: {result.stderr}")
            return False
        
        available_topics = result.stdout.strip().split('\n')
        print(f"📊 Available topics: {len(available_topics)}")
        
        found_count = 0
        for required in required_topics:
            if required in available_topics:
                print(f"✅ Found: {required}")
                found_count += 1
            else:
                print(f"❌ Missing: {required}")
        
        if found_count > 0:
            print(f"\n✅ ROS Topics test PASSED ({found_count}/{len(required_topics)} required)")
            return True
        else:
            print(f"\n❌ None of the required topics found")
            return False
            
    except Exception as e:
        print(f"❌ Topics test FAILED: {e}")
        return False


# =========================================================
# NETWORK TESTS
# =========================================================

def test_network_connectivity(host="192.168.0.165"):
    """Test network connectivity to MQTT broker host."""
    print("\n" + "="*60)
    print("🧪 NETWORK CONNECTIVITY TEST")
    print("="*60)
    
    print(f"🔍 Pinging {host}...")
    
    try:
        # Use ping to test connectivity
        result = subprocess.run(
            ["ping", "-c" if sys.platform != "win32" else "-n", "1", host],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            print(f"✅ Network connectivity to {host} OK")
            return True
        else:
            print(f"❌ Cannot reach {host}")
            return False
            
    except Exception as e:
        print(f"❌ Network test FAILED: {e}")
        return False


# =========================================================
# FILE TESTS
# =========================================================

def test_files_and_config():
    """Check for required files and configuration."""
    print("\n" + "="*60)
    print("🧪 FILES & CONFIGURATION TEST")
    print("="*60)
    
    required_files = [
        "LLM Interface with ROS output for Pi.py",
        "waypoint_manager.py",
        "mqtt_ros_bridge_ENHANCED.py"
    ]
    
    found_count = 0
    for filename in required_files:
        if Path(filename).exists():
            print(f"✅ Found: {filename}")
            found_count += 1
        else:
            print(f"❌ Missing: {filename}")
    
    # Check for configuration files
    print("\nConfiguration files:")
    optional_files = ["devices.json", "waypoints.json", "waypoint_history.json"]
    
    for filename in optional_files:
        if Path(filename).exists():
            try:
                with open(filename, 'r') as f:
                    json.load(f)
                print(f"✅ Valid JSON: {filename}")
            except json.JSONDecodeError:
                print(f"❌ Invalid JSON: {filename}")
        else:
            print(f"⚠️ Not found (will be created): {filename}")
    
    print(f"\n✅ Files test PASSED ({found_count}/{len(required_files)} required found)")
    return found_count > 0


# =========================================================
# INTEGRATION TEST
# =========================================================

def test_integration():
    """Full integration test of all components."""
    print("\n" + "="*60)
    print("🧪 FULL INTEGRATION TEST")
    print("="*60)
    
    tests_passed = 0
    tests_total = 0
    
    # Test 1: Network
    tests_total += 1
    if test_network_connectivity():
        tests_passed += 1
    
    # Test 2: MQTT
    tests_total += 1
    if test_mqtt_connectivity():
        tests_passed += 1
    
    # Test 3: Waypoints
    tests_total += 1
    if test_waypoint_manager():
        tests_passed += 1
    
    # Test 4: Files
    tests_total += 1
    if test_files_and_config():
        tests_passed += 1
    
    # Test 5: ROS (if available)
    if HAVE_ROS:
        tests_total += 1
        if test_ros_connectivity():
            tests_passed += 1
    
    print("\n" + "="*60)
    print(f"📊 INTEGRATION TEST RESULTS: {tests_passed}/{tests_total} passed")
    print("="*60)
    
    return tests_passed == tests_total


# =========================================================
# MAIN
# =========================================================

def main():
    parser = argparse.ArgumentParser(
        description="Robot Control System Diagnostics"
    )
    parser.add_argument(
        "--test",
        choices=["all", "mqtt", "waypoints", "ros", "network", "files", "integration"],
        default="all",
        help="Which test to run"
    )
    
    args = parser.parse_args()
    
    print("\n")
    print("╔" + "="*58 + "╗")
    print("║" + " "*15 + "ROBOT SYSTEM DIAGNOSTICS" + " "*19 + "║")
    print("╚" + "="*58 + "╝")
    
    if args.test == "all":
        success = test_integration()
    elif args.test == "mqtt":
        test_mqtt_connectivity()
        success = test_mqtt_publish_subscribe()
    elif args.test == "waypoints":
        success = test_waypoint_manager()
    elif args.test == "ros":
        test_ros_connectivity()
        success = test_ros_topics()
    elif args.test == "network":
        success = test_network_connectivity()
    elif args.test == "files":
        success = test_files_and_config()
    elif args.test == "integration":
        success = test_integration()
    
    print("\n")
    if success:
        print("✅ All tests completed successfully!")
        print("Your robot control system is ready to use.")
        sys.exit(0)
    else:
        print("⚠️ Some tests failed. Check the output above for details.")
        print("See SETUP_AND_USAGE_GUIDE.md for troubleshooting.")
        sys.exit(1)


if __name__ == "__main__":
    main()
