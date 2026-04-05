#!/usr/bin/env python3
"""
Test script for IK-enabled robot controller
Demonstrates moving to specific XYZ positions
"""

from raspberry_pi_controller import RobotArmController
import time

def main():
    # Initialize controller
    robot = RobotArmController(port='/dev/ttyACM0', baudrate=115200)
    
    # Connect to Arduino
    print("Connecting to robot...")
    if not robot.connect():
        print("Failed to connect. Exiting.")
        return
    
    try:
        # Display current position
        print("\nCurrent Position:")
        robot.print_position()
        
        # Test 1: Move to a specific XYZ position
        print("\n" + "="*60)
        print("TEST 1: Move to position (10, 15, 40) cm")
        print("="*60)
        robot.move_to_position(10, 15, 40, smooth=True, display_progress=True)
        time.sleep(1)
        
        # Test 2: Move to another position
        print("\n" + "="*60)
        print("TEST 2: Move to position (-5, 20, 50) cm")
        print("="*60)
        robot.move_to_position(-5, 20, 50, smooth=True, display_progress=True)
        time.sleep(1)
        
        # Test 3: Move to a centered position
        print("\n" + "="*60)
        print("TEST 3: Move to position (0, 25, 45) cm")
        print("="*60)
        robot.move_to_position(0, 25, 45, smooth=True, display_progress=True)
        time.sleep(1)
        
        # Display final position
        print("\nFinal Position:")
        robot.print_position()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        # Clean up
        robot.disconnect()
        print("Test complete!")

if __name__ == "__main__":
    main()
