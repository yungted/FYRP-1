#!/usr/bin/env python3
"""
Waypoint Manager Command-Line Interface
========================================
Utility script for safely inspecting and managing waypoints.

Usage:
  python3 -m waypoints.waypoint_manager_cli list          # List all waypoints
  python3 -m waypoints.waypoint_manager_cli show           # Show detailed info (alias for list)
  python3 -m waypoints.waypoint_manager_cli count          # Show total count
  python3 -m waypoints.waypoint_manager_cli clear          # Delete ALL waypoints (asks for confirmation)
  python3 -m waypoints.waypoint_manager_cli delete <name>  # Delete specific waypoint
  python3 -m waypoints.waypoint_manager_cli validate       # Check waypoint file integrity
  python3 -m waypoints.waypoint_manager_cli repair         # Fix malformed waypoints
"""

import sys
import json
import os
from .waypoint_manager import WaypointManager

def print_header(title):
    """Print a formatted header."""
    print(f"\n{'='*70}")
    print(f" {title}")
    print(f"{'='*70}")

def list_waypoints(manager):
    """Display all waypoints in detail."""
    print_header("📍 WAYPOINT INVENTORY")
    manager.print_all_waypoints()

def show_count(manager):
    """Show total waypoint count."""
    count = manager.get_waypoint_count()
    print_header("📊 WAYPOINT COUNT")
    print(f"\n   Total waypoints saved: {count}\n")

def validate_waypoints(manager):
    """Validate waypoint file integrity."""
    print_header("✓ WAYPOINT VALIDATION")
    
    waypoints = manager.get_all_waypoints()
    issues = []
    
    for name, wp in waypoints.items():
        # Check required fields
        required = ["x", "y", "z", "w"]
        for field in required:
            if field not in wp:
                issues.append(f"  ❌ '{name}': Missing '{field}' field")
        
        # Check coordinate types
        for field in required:
            if field in wp and not isinstance(wp[field], (int, float)):
                issues.append(f"  ❌ '{name}': '{field}' is not a number")
        
        # Check description
        if "description" not in wp:
            issues.append(f"  ⚠️  '{name}': Missing 'description'")
    
    if not issues:
        print("\n   ✅ All waypoints are valid!\n")
    else:
        print("\n   Issues found:")
        for issue in issues:
            print(issue)
        print()

def repair_waypoints(manager):
    """Repair malformed waypoints by fixing them."""
    print_header("🔧 WAYPOINT REPAIR")
    
    waypoints = manager.get_all_waypoints()
    repaired = 0
    
    for name, wp in waypoints.items():
        # Fix missing fields
        if "x" not in wp:
            wp["x"] = 0.0
            repaired += 1
        if "y" not in wp:
            wp["y"] = 0.0
            repaired += 1
        if "z" not in wp:
            wp["z"] = 0.0
            repaired += 1
        if "w" not in wp:
            wp["w"] = 1.0
            repaired += 1
        if "description" not in wp:
            wp["description"] = f"Location at ({wp.get('x', 0):.2f}, {wp.get('y', 0):.2f})"
            repaired += 1
    
    if repaired > 0:
        manager.save_waypoints()
        print(f"\n   ✅ Repaired {repaired} field(s)\n")
    else:
        print(f"\n   ✅ No repairs needed\n")

def delete_waypoint(manager, name):
    """Delete a specific waypoint."""
    print_header(f"🗑️  DELETE WAYPOINT: '{name}'")
    
    # Confirm
    response = input(f"\n   Are you sure you want to delete '{name}'? (yes/no): ").strip().lower()
    if response not in ("yes", "y"):
        print("   ❌ Cancelled\n")
        return
    
    success = manager.remove_waypoint(name)
    if success:
        print(f"   ✅ Deleted '{name}'\n")
    else:
        print(f"   ❌ Failed to delete '{name}'\n")

def clear_all_waypoints(manager):
    """Delete all waypoints with multiple confirmations."""
    count = manager.get_waypoint_count()
    
    if count == 0:
        print_header("🗑️  CLEAR ALL WAYPOINTS")
        print("\n   ℹ️  No waypoints to delete\n")
        return
    
    print_header("🗑️  CLEAR ALL WAYPOINTS - DESTRUCTIVE OPERATION")
    print(f"\n   ⚠️  WARNING: This will delete ALL {count} waypoint(s)")
    print("   This action CANNOT be undone!\n")
    
    # Show waypoints first
    print("   Current waypoints:")
    for name in manager.list_waypoints():
        print(f"     - {name}")
    print()
    
    # First confirmation
    response1 = input("   Type 'DELETE' to confirm deletion: ").strip()
    if response1 != "DELETE":
        print("   ❌ Cancelled\n")
        return
    
    # Second confirmation
    response2 = input("   Type the number of waypoints to confirm ({}): ".format(count)).strip()
    try:
        if int(response2) != count:
            print("   ❌ Incorrect count - cancelled\n")
            return
    except ValueError:
        print("   ❌ Invalid input - cancelled\n")
        return
    
    # Final confirmation
    response3 = input("   This is your LAST chance. Type 'YES, DELETE ALL' to proceed: ").strip()
    if response3 != "YES, DELETE ALL":
        print("   ❌ Cancelled\n")
        return
    
    # Execute deletion
    success = manager.clear_all_waypoints(confirm=True)
    if success:
        print(f"   ✅ DELETED: All {count} waypoint(s) have been removed\n")
    else:
        print("   ❌ Failed to delete waypoints\n")

def main():
    """Main CLI interface."""
    if len(sys.argv) < 2:
        print(__doc__)
        return
    
    command = sys.argv[1].lower()
    manager = WaypointManager()
    
    if command in ("list", "show"):
        list_waypoints(manager)
    elif command == "count":
        show_count(manager)
    elif command == "validate":
        validate_waypoints(manager)
    elif command == "repair":
        repair_waypoints(manager)
    elif command == "clear":
        clear_all_waypoints(manager)
    elif command == "delete":
        if len(sys.argv) < 3:
            print("Usage: python3 -m waypoints.waypoint_manager_cli delete <waypoint_name>")
            return
        delete_waypoint(manager, sys.argv[2])
    else:
        print(f"Unknown command: {command}")
        print(__doc__)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Cancelled by user\n")
    except Exception as e:
        print(f"\n❌ Error: {e}\n")
