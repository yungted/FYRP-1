#!/usr/bin/env python3
"""
Waypoint Manager Module
======================
Handles storage, retrieval, visualization, and management of robot navigation waypoints.

Features:
- Store waypoints with semantic names (e.g., "kitchen", "living_room")
- Persist waypoints to JSON file for consistency across sessions
- Track last visited waypoint and navigation history
- Generate user-friendly descriptions for LLM context
- Support for RVIZ integration and pose estimation
- Error handling for malformed waypoints
"""

import json
import os
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import math

# Default waypoint storage file (in same directory as this module)
WAYPOINTS_FILE = os.path.join(os.path.dirname(__file__), "waypoints.json")
WAYPOINT_HISTORY_FILE = os.path.join(os.path.dirname(__file__), "waypoint_history.json")

class WaypointManager:
    """
    Manages robot waypoints with persistent storage and context-aware retrieval.
    
    Data Structure:
    {
        "waypoints": {
            "waypoint_name": {
                "x": float,
                "y": float,
                "z": float (quaternion z),
                "w": float (quaternion w),
                "description": str,
                "created_at": str (ISO format),
                "visited_count": int,
                "last_visited": str (ISO format)
            },
            ...
        },
        "metadata": {
            "last_updated": str,
            "total_waypoints": int
        }
    }
    """
    
    def __init__(self, waypoints_file: str = None):
        """
        Initialize waypoint manager.
        
        Args:
            waypoints_file: Path to persistent waypoint storage JSON file (defaults to WAYPOINTS_FILE)
        """
        self.waypoints_file = waypoints_file or WAYPOINTS_FILE
        self.waypoints: Dict[str, dict] = {}
        self.metadata: Dict = {
            "last_updated": None,
            "total_waypoints": 0
        }
        self.load_waypoints()
    
    def load_waypoints(self) -> bool:
        """
        Load waypoints from persistent storage.
        
        Returns:
            bool: True if successful, False if file doesn't exist or is malformed
        """
        if not os.path.exists(self.waypoints_file):
            print(f"📁 No waypoint file found at {self.waypoints_file}. Starting fresh.")
            self.waypoints = {}
            self.metadata = {"last_updated": None, "total_waypoints": 0}
            return False
        
        try:
            with open(self.waypoints_file, "r") as f:
                data = json.load(f)
            
            self.waypoints = data.get("waypoints", {})
            self.metadata = data.get("metadata", {
                "last_updated": None,
                "total_waypoints": len(self.waypoints)
            })
            
            # Validate all waypoints have required fields
            for name, wp in self.waypoints.items():
                if not self._validate_waypoint(wp):
                    print(f"⚠️ Malformed waypoint '{name}', fixing...")
                    self._fix_waypoint(wp)
            
            print(f"✅ Loaded {len(self.waypoints)} waypoints from {self.waypoints_file}")
            return True
            
        except json.JSONDecodeError as e:
            print(f"❌ Failed to parse waypoint file: {e}")
            return False
        except Exception as e:
            print(f"❌ Error loading waypoints: {e}")
            return False
    
    def save_waypoints(self) -> bool:
        """
        Save waypoints to persistent storage.
        
        Returns:
            bool: True if successful
        """
        try:
            data = {
                "waypoints": self.waypoints,
                "metadata": {
                    "last_updated": datetime.now().isoformat(),
                    "total_waypoints": len(self.waypoints)
                }
            }
            
            with open(self.waypoints_file, "w") as f:
                json.dump(data, f, indent=2)
            
            print(f"💾 Saved {len(self.waypoints)} waypoints to {self.waypoints_file}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to save waypoints: {e}")
            return False
    
    def _validate_waypoint(self, waypoint: dict) -> bool:
        """
        Validate that a waypoint has all required fields.
        
        Args:
            waypoint: Waypoint dictionary to validate
            
        Returns:
            bool: True if valid
        """
        required_fields = ["x", "y", "z", "w"]
        for field in required_fields:
            if field not in waypoint or not isinstance(waypoint[field], (int, float)):
                return False
        return True
    
    def _fix_waypoint(self, waypoint: dict) -> None:
        """
        Attempt to fix malformed waypoint by setting defaults.
        
        Args:
            waypoint: Waypoint dictionary to fix (modified in-place)
        """
        if "x" not in waypoint:
            waypoint["x"] = 0.0
        if "y" not in waypoint:
            waypoint["y"] = 0.0
        if "z" not in waypoint:
            waypoint["z"] = 0.0  # quaternion z (no rotation)
        if "w" not in waypoint:
            waypoint["w"] = 1.0  # quaternion w (identity)
        if "description" not in waypoint:
            waypoint["description"] = "Unknown location"
        if "created_at" not in waypoint:
            waypoint["created_at"] = datetime.now().isoformat()
        if "visited_count" not in waypoint:
            waypoint["visited_count"] = 0
        if "last_visited" not in waypoint:
            waypoint["last_visited"] = None
    
    def add_waypoint(self, name: str, x: float, y: float, z: float = 0.0, w: float = 1.0, 
                     description: str = "") -> bool:
        """
        Add a new waypoint or update existing one.
        
        Args:
            name: Semantic name for the waypoint (e.g., "kitchen", "office")
            x: X coordinate in map frame
            y: Y coordinate in map frame
            z: Quaternion z component (rotation)
            w: Quaternion w component (identity rotation = w=1, z=0)
            description: Human-readable description of the location
            
        Returns:
            bool: True if successful
        """
        name_normalized = name.lower().strip()
        
        # Validation
        if not name_normalized:
            print("❌ Waypoint name cannot be empty")
            return False
        
        if len(name_normalized) > 32:
            print(f"❌ Waypoint name too long (max 32 chars): {name}")
            return False
        
        # Sanitize name (allow alphanumeric, underscores, hyphens)
        if not all(c.isalnum() or c in "-_" for c in name_normalized):
            print(f"❌ Waypoint name contains invalid characters: {name}")
            print(f"   Use only letters, numbers, hyphens, and underscores")
            return False
        
        try:
            x, y, z, w = float(x), float(y), float(z), float(w)
        except (TypeError, ValueError) as e:
            print(f"❌ Invalid coordinate values: {e}")
            return False
        
        # Normalize quaternion
        q_norm = math.sqrt(z*z + w*w)
        if q_norm > 0:
            z /= q_norm
            w /= q_norm
        
        # Create/update waypoint
        is_new = name_normalized not in self.waypoints
        self.waypoints[name_normalized] = {
            "x": round(x, 4),
            "y": round(y, 4),
            "z": round(z, 4),
            "w": round(w, 4),
            "description": description.strip() or f"Location at ({x:.2f}, {y:.2f})",
            "created_at": self.waypoints.get(name_normalized, {}).get("created_at", 
                                                                      datetime.now().isoformat()),
            "visited_count": self.waypoints.get(name_normalized, {}).get("visited_count", 0),
            "last_visited": self.waypoints.get(name_normalized, {}).get("last_visited")
        }
        
        self.save_waypoints()
        
        if is_new:
            print(f"✅ Added new waypoint: '{name_normalized}' at ({x:.2f}, {y:.2f})")
        else:
            print(f"✅ Updated waypoint: '{name_normalized}' at ({x:.2f}, {y:.2f})")
        
        return True
    
    def get_waypoint(self, name: str) -> Optional[Dict]:
        """
        Retrieve a waypoint by name (case-insensitive).
        
        Args:
            name: Waypoint name
            
        Returns:
            Waypoint dict or None if not found
        """
        name_normalized = name.lower().strip()
        return self.waypoints.get(name_normalized)
    
    def get_all_waypoints(self) -> Dict[str, Dict]:
        """
        Get all stored waypoints.
        
        Returns:
            Dictionary of all waypoints
        """
        return self.waypoints.copy()
    
    def list_waypoints(self) -> List[str]:
        """
        Get list of all waypoint names.
        
        Returns:
            List of waypoint names
        """
        return list(self.waypoints.keys())
    
    def remove_waypoint(self, name: str) -> bool:
        """
        Delete a waypoint.
        
        Args:
            name: Waypoint name to delete
            
        Returns:
            bool: True if successful
        """
        name_normalized = name.lower().strip()
        
        if name_normalized not in self.waypoints:
            print(f"❌ Waypoint '{name_normalized}' not found")
            return False
        
        del self.waypoints[name_normalized]
        self.save_waypoints()
        print(f"🗑️ Deleted waypoint: '{name_normalized}'")
        return True
    
    def mark_visited(self, name: str) -> bool:
        """
        Mark a waypoint as visited (increments counter, updates timestamp).
        
        Args:
            name: Waypoint name
            
        Returns:
            bool: True if successful
        """
        name_normalized = name.lower().strip()
        
        if name_normalized not in self.waypoints:
            print(f"⚠️ Waypoint '{name_normalized}' not found for visit tracking")
            return False
        
        wp = self.waypoints[name_normalized]
        wp["visited_count"] = wp.get("visited_count", 0) + 1
        wp["last_visited"] = datetime.now().isoformat()
        
        self.save_waypoints()
        print(f"📍 Marked '{name_normalized}' as visited ({wp['visited_count']} times)")
        return True
    
    def get_context_for_llm(self) -> str:
        """
        Generate human-readable waypoint list for LLM system prompt.
        
        Returns:
            Formatted string describing all waypoints
        """
        if not self.waypoints:
            return "No waypoints defined yet. User can say 'save this location as <name>' to create waypoints."
        
        lines = ["AVAILABLE WAYPOINTS:"]
        for name, wp in self.waypoints.items():
            x, y = wp.get("x", 0), wp.get("y", 0)
            desc = wp.get("description", "Unknown")
            visited = wp.get("visited_count", 0)
            lines.append(f"  - '{name}': {desc} (at {x:.2f}, {y:.2f}) [visited {visited} times]")
        
        lines.append("\nTo navigate to a waypoint, user can say: 'go to <waypoint_name>'")
        return "\n".join(lines)
    
    def find_nearby_waypoint(self, x: float, y: float, max_distance: float = 0.5) -> Optional[str]:
        """
        Find the closest waypoint to a given position.
        
        Args:
            x: X coordinate
            y: Y coordinate
            max_distance: Maximum distance to consider (meters)
            
        Returns:
            Name of closest waypoint or None if none within distance
        """
        closest_name = None
        closest_dist = float('inf')
        
        for name, wp in self.waypoints.items():
            dist = math.sqrt((x - wp["x"])**2 + (y - wp["y"])**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_name = name
        
        if closest_dist <= max_distance:
            return closest_name
        
        return None
    
    def get_waypoint_as_ros_pose(self, name: str) -> Optional[Tuple[float, float, float, float]]:
        """
        Get a waypoint in ROS pose format (x, y, z, w for quaternion).
        
        Args:
            name: Waypoint name
            
        Returns:
            Tuple of (x, y, z, w) or None if not found
        """
        wp = self.get_waypoint(name)
        if not wp:
            return None
        
        return (wp["x"], wp["y"], wp["z"], wp["w"])
    
    def clear_all_waypoints(self, confirm: bool = False) -> bool:
        """
        WARNING: Delete all waypoints (requires explicit confirmation).
        
        Args:
            confirm: Must be True to actually delete waypoints
        
        Returns:
            bool: True if successful
        """
        if not confirm:
            count = len(self.waypoints)
            print(f"⚠️ NOT CONFIRMED: Would delete {count} waypoints. "
                  f"Call clear_all_waypoints(confirm=True) to actually delete.")
            return False
        
        count = len(self.waypoints)
        self.waypoints = {}
        self.save_waypoints()
        print(f"🗑️ DELETED: All {count} waypoints cleared from {self.waypoints_file}")
        return True
    
    def get_waypoint_count(self) -> int:
        """
        Get total number of saved waypoints.
        
        Returns:
            int: Number of waypoints
        """
        return len(self.waypoints)
    
    def print_all_waypoints(self) -> None:
        """
        Print all waypoints in a human-readable format for debugging.
        Helps identify hallucinated waypoints.
        """
        count = len(self.waypoints)
        print(f"\n{'='*60}")
        print(f"📍 WAYPOINT INVENTORY ({count} total)")
        print(f"{'='*60}")
        
        if not self.waypoints:
            print("   [No waypoints saved]")
        else:
            for idx, (name, wp) in enumerate(self.waypoints.items(), 1):
                print(f"\n   {idx}. {name}")
                print(f"      Position: ({wp.get('x'):.4f}, {wp.get('y'):.4f})")
                print(f"      Rotation: z={wp.get('z'):.4f}, w={wp.get('w'):.4f}")
                print(f"      Description: {wp.get('description', 'N/A')}")
                print(f"      Created: {wp.get('created_at', 'N/A')}")
                print(f"      Visited: {wp.get('visited_count', 0)} times")
        
        print(f"{'='*60}\n")


# ====================================================================
# Utility Functions for LLM Command Interpretation
# ====================================================================

def extract_waypoint_name_from_command(command: str) -> Optional[str]:
    """
    Extract waypoint name from user commands like:
    - "go to kitchen"
    - "navigate to the living room"
    - "take me to kitchen"
    
    Args:
        command: User voice command
        
    Returns:
        Extracted waypoint name or None
    """
    command_lower = command.lower()
    
    # Keywords that introduce a waypoint name
    keywords = ["go to", "navigate to", "take me to", "bring me to", "head to", "drive to"]
    
    for keyword in keywords:
        if keyword in command_lower:
            # Extract everything after the keyword
            parts = command_lower.split(keyword)
            if len(parts) > 1:
                waypoint_name = parts[-1].strip()
                # Remove common filler words
                filler_words = ["the", "my", "our"]
                for word in filler_words:
                    if waypoint_name.startswith(word + " "):
                        waypoint_name = waypoint_name[len(word)+1:].strip()
                # Remove trailing punctuation
                waypoint_name = waypoint_name.rstrip(".,!?")
                return waypoint_name
    
    return None


def generate_waypoint_navigation_prompt(waypoint_name: str) -> str:
    """
    Generate a high-level navigation prompt for the LLM.
    
    Args:
        waypoint_name: Name of destination waypoint
        
    Returns:
        Formatted navigation instruction
    """
    return f"User wants to navigate to waypoint '{waypoint_name}'. Use navigate_to_waypoint action with the waypoint name."


if __name__ == "__main__":
    # Test the waypoint manager
    manager = WaypointManager()
    
    # Add some test waypoints
    manager.add_waypoint("kitchen", 1.97, 12.24, 1.0, 0.0, "Main kitchen area")
    manager.add_waypoint("living_room", 2.11, 24.33, 1.0, 0.0, "Living room with furniture")
    manager.add_waypoint("bedroom", -0.92, 28.65, 0.0, 1.0, "Master bedroom")
    
    # Test retrieval
    print("\n📋 All waypoints:")
    for name in manager.list_waypoints():
        wp = manager.get_waypoint(name)
        print(f"  {name}: {wp}")
    
    # Test LLM context
    print("\n" + manager.get_context_for_llm())
    
    # Test command parsing
    test_commands = [
        "go to kitchen",
        "navigate to the living room",
        "take me to bedroom"
    ]
    print("\n🎤 Command parsing test:")
    for cmd in test_commands:
        name = extract_waypoint_name_from_command(cmd)
        print(f"  '{cmd}' -> waypoint: '{name}'")
