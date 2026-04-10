#!/usr/bin/env python3
"""
Test script to demonstrate forward kinematics integration
"""

from raspberry_pi_controller import RobotArmController
import time


def main():
    # Initialize controller
    from raspberry_pi_controller import DEFAULT_PORT
    robot = RobotArmController(port=DEFAULT_PORT, baudrate=115200, step_delay=0.015)
    
    # Connect to Arduino
    print("Connecting to Arduino...")
    if not robot.connect():
        print("Failed to connect. Check the port and try again.")
        return
    
    try:
        # Show initial position
        print("\n=== Initial Position ===")
        robot.print_position()
        
        time.sleep(2)
        
        # Move to a position and show FK
        print("\n=== Moving to position 1 ===")
        robot.send_command({
            'b': 45,
            's': 30,
            'e': 60,
            'w': 45
        }, smooth=True, display_progress=True)
        
        time.sleep(0.5)
        robot.print_position()
        
        time.sleep(2)
        
        # Move to another position
        print("\n=== Moving to position 2 ===")
        robot.send_command({
            'b': 90,
            's': 45,
            'e': 90,
            'w': 30
        }, smooth=True, display_progress=True)
        
        time.sleep(0.5)
        robot.print_position()
        
        time.sleep(2)
        
        # Get position directly
        print("\n=== Getting current XYZ ===")
        x, y, z = robot.get_end_effector_position()
        print(f"End effector at: X={x:.2f}, Y={y:.2f}, Z={z:.2f} cm")
        
        # Return home
        print("\n=== Returning to home ===")
        robot.home_position(smooth=True, display_progress=True)
        time.sleep(0.5)
        robot.print_position()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nDisconnecting...")
        robot.disconnect()
        print("Done!")


if __name__ == "__main__":
    main()
