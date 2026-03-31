#!/usr/bin/env python3
"""
Test script for smooth motion control
Demonstrates checking current angles and displaying real-time movement
"""

from raspberry_pi_controller import RobotArmController
import time


def main():
    # Initialize controller with custom step delay (adjust for speed)
    # Lower step_delay = faster movement
    robot = RobotArmController(port='COM4', baudrate=115200, step_delay=0.015)
    
    # Connect to Arduino
    print("Connecting to Arduino...")
    if not robot.connect():
        print("Failed to connect. Check the port and try again.")
        return
    
    try:
        # Get current angles before starting
        print("\n=== Initial Position ===")
        current = robot.get_current_angles()
        print(f"Current angles: {current}")
        
        # Example 1: Move multiple servos with smooth motion and progress display
        print("\n\n=== Test 1: Move multiple servos ===")
        robot.send_command({
            'b': 90,   # Base to 90 degrees
            's': 90,   # Shoulder to 45 degrees
            'e': 90,   # Elbow to 90 degrees
            'w': 90,   # Wrist to 60 degrees
            't': 90,   # Twist to 45 degrees
            'g': 40    # Gripper half open
        }, smooth=True, display_progress=True)
        
        time.sleep(1)
        
        # Example 2: Move single servo
        print("\n\n=== Test 2: Move single servo (Gripper) ===")
        robot.move_servo('g', 80, smooth=True, display_progress=True)
        
        time.sleep(1)
        
        # Return to home position
        # print("\n\n=== Returning to home position ===")
        robot.home_position(smooth=True, display_progress=True)
        
        # print("\n\n=== All tests complete! ===")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\nDisconnecting...")
        robot.disconnect()
        print("Done!")


if __name__ == "__main__":
    main()
