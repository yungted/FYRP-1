"""
Waypoint Management Package
============================
Handles robot navigation waypoints with persistent storage and retrieval.

Imports:
    from waypoints.waypoint_manager import WaypointManager, extract_waypoint_name_from_command
"""

from .waypoint_manager import WaypointManager, extract_waypoint_name_from_command, generate_waypoint_navigation_prompt

__all__ = [
    'WaypointManager',
    'extract_waypoint_name_from_command',
    'generate_waypoint_navigation_prompt'
]
