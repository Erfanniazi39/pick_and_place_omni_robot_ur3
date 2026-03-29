#!/usr/bin/env python3
"""
Diagnostic script to check map loading and publishing status.
Run this while Nav2 is running to verify the map is being published.
"""
import subprocess
import sys
import time

def run_command(cmd):
    """Run a shell command and return output"""
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return result.stdout + result.stderr

def main():
    print("=" * 60)
    print("NAV2 MAP DIAGNOSTIC REPORT")
    print("=" * 60)
    
    # Check 1: Map file existence
    print("\n[1] Checking map file existence...")
    map_yaml = "/home/legionaire/ros2_ws/src/omni_navigation/maps/my_map.yaml"
    map_pgm = "/home/legionaire/ros2_ws/src/omni_navigation/maps/my_map.pgm"
    
    import os
    yaml_exists = os.path.exists(map_yaml)
    pgm_exists = os.path.exists(map_pgm)
    
    print(f"    Map YAML: {map_yaml}")
    print(f"      └─ {'✓ EXISTS' if yaml_exists else '✗ MISSING'}")
    print(f"    Map PGM: {map_pgm}")
    print(f"      └─ {'✓ EXISTS' if pgm_exists else '✗ MISSING'}")
    
    if yaml_exists:
        with open(map_yaml, 'r') as f:
            print(f"    YAML content:\n{f.read()}")
    
    # Check 2: ROS2 topics
    print("\n[2] Checking ROS2 topics...")
    topics = run_command("ros2 topic list 2>/dev/null | grep -E '(map|costmap)'")
    if topics.strip():
        print("    Topics related to map/costmap:")
        for line in topics.strip().split('\n'):
            print(f"      └─ {line}")
    else:
        print("    ✗ NO map/costmap topics found!")
    
    # Check 3: /map topic details
    print("\n[3] Checking /map topic...")
    map_info = run_command("ros2 topic info /map 2>/dev/null")
    if "Unknown topic" not in map_info and "ERROR" not in map_info:
        print("    ✓ /map topic EXISTS")
        print(f"    {map_info}")
    else:
        print("    ✗ /map topic NOT FOUND or ERROR")
    
    # Check 4: Global costmap layers
    print("\n[4] Checking nav2_map_server node...")
    nodes = run_command("ros2 node list 2>/dev/null | grep -E '(map_server|global_costmap|static_layer)'")
    if nodes.strip():
        print("    Related nodes found:")
        for line in nodes.strip().split('\n'):
            print(f"      └─ {line}")
    else:
        print("    ⚠ No specific map_server/costmap nodes found")
    
    # Check 5: Try to subscribe to /map for 2 seconds
    print("\n[5] Attempting to receive /map message (listening for 2 seconds)...")
    map_msg = run_command("timeout 2 ros2 topic echo /map --once 2>/dev/null | head -20")
    if map_msg.strip() and "Unknown topic" not in map_msg:
        print("    ✓ Received /map message:")
        for line in map_msg.strip().split('\n')[:10]:
            print(f"      {line}")
    else:
        print("    ✗ No /map message received or topic doesn't exist")
    
    # Check 6: Nav2 params
    print("\n[6] Checking static_layer configuration...")
    config_file = "/home/legionaire/ros2_ws/src/omni_navigation/config/nav2_params.yaml"
    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            content = f.read()
            if 'static_layer' in content:
                print("    ✓ static_layer found in config")
                if 'map_subscribe_transient_local' in content:
                    print("    ✓ map_subscribe_transient_local enabled")
            else:
                print("    ✗ static_layer NOT in config")
    
    print("\n" + "=" * 60)
    print("DIAGNOSTIC COMPLETE")
    print("=" * 60)
    print("\nIf /map topic is NOT found:")
    print("  1. Check Nav2 launch output for errors")
    print("  2. Verify map file paths are absolute and correct")
    print("  3. Check: ros2 node list | grep map")
    print("  4. Check: ros2 launch nav2_bringup localization_launch.py with map:=<path>")

if __name__ == '__main__':
    main()
