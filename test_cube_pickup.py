#!/usr/bin/env python3
"""
FAST Pick and Place Test - 2cm Cube
Cube dimensions: 2cm x 2cm x 2cm
Pick from: (-10, 10, 0) floor position
Place at: (10, 10, 0) floor position

SPEED OPTIMIZATIONS:
- Linear interpolation (faster than cubic)
- Reduced waypoints: 3 approach points (was 5), 8 transfer (was 15)
- Reduced heights: 5cm approach/retract (was 8cm)
- Fast execution: dt=0.005s (was 0.02s) = 4x faster
- Expected total waypoints: ~24 (was 52)
- Estimated time: ~0.12s movement (was ~1s)
"""

from raspberry_pi_controller import RobotArmController

def main():
    # Initialize
    robot = RobotArmController(port='COM4')
    
    print("="*70)
    print("CUBE PICK AND PLACE TEST")
    print("="*70)
    print("Cube: 2cm x 2cm x 2cm")
    print("Pick from: (-10, 10, 0) - floor position")
    print("Place at: (10, 10, 0) - floor position")
    print("="*70)
    
    # Connect to robot
    if not robot.connect():
        print("Failed to connect to robot!")
        return
    
    print("\n✓ Connected to robot")

    # Angles are already fresh from the polling done inside connect()
    current_angles = robot.get_current_angles()
    print(f"\n📡 Current position from robot:")
    print(f"   Angles: b={current_angles['b']}° s={current_angles['s']}° "
          f"e={current_angles['e']}° w={current_angles['w']}°")
    current_pos = robot.get_end_effector_position()
    print(f"   End-effector: ({current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f})")
    cube_height = 2.0  # cm
    grasp_height = cube_height / 2  # Grasp at center of cube (1cm)
    
    # Pick position: cube center at (-10, 10, 1)
    pick_x = -10.0
    pick_y = 10.0
    pick_z = grasp_height  # 1cm - center of 2cm cube
    
    # Place position: cube center at (10, 10, 1)
    place_x = 10.0
    place_y = 10.0
    place_z = grasp_height  # 1cm - center of 2cm cube
    
    print(f"\n📦 Cube Specifications:")
    print(f"   Dimensions: 2cm x 2cm x 2cm")
    print(f"   Floor position at pick: ({pick_x}, {pick_y}, 0)")
    print(f"   Grasp height (center): {grasp_height}cm")
    
    print(f"\n📍 Pick Position:")
    print(f"   Position: ({pick_x}, {pick_y}, {pick_z})")
    
    print(f"\n📍 Place Position:")
    print(f"   Position: ({place_x}, {place_y}, {place_z})")
    
    # Safety margins - reduced for faster operation
    approach_height = 5.0  # cm above grasp point (reduced from 8)
    retract_height = 5.0   # cm to lift after grasp (reduced from 8)
    
    print(f"\n⚙️ Settings:")
    print(f"   Approach height: {approach_height}cm")
    print(f"   Retract height: {retract_height}cm")
    print(f"   Gripper open: 30° (fully open)")
    print(f"   Gripper close: 80° (fully closed)")
    print(f"   Joint-space interpolation (30 steps/segment)")
    print(f"   dt: 70ms/waypoint → ~2.1s/segment")
    
    # Ask for confirmation
    print("\n" + "="*70)
    response = input("Ready to execute? (yes/no): ")
    if response.lower() != 'yes':
        print("Operation cancelled")
        robot.disconnect()
        return
    
    # Execute
    print("\n" + "="*70)
    print("EXECUTING PICK AND PLACE")
    print("="*70)
    
    try:
        success = robot.execute_pick_and_place(
            pick_pos=(pick_x, pick_y, pick_z),
            place_pos=(place_x, place_y, place_z),
            approach_height=approach_height,
            retract_height=retract_height,
            gripper_open=30,
            gripper_close=80,
            steps_per_segment=30,
            dt=0.07,       # transit phases (get-to-pos, transfer)
            vertical_dt=0.14,  # descend/retract/drop — 2× slower
            display_progress=True
        )
        
        if success:
            print("\n" + "="*70)
            print("✓ PICK AND PLACE COMPLETED SUCCESSFULLY!")
            print("="*70)
            
            # Verify final position
            final_pos = robot.get_end_effector_position()
            print(f"\n📍 Final position: ({final_pos[0]:.1f}, {final_pos[1]:.1f}, {final_pos[2]:.1f})")

            # Return home after drop
            print("\n🏠 Returning to home position...")
            robot.home_position(smooth=True, display_progress=True)
            home_pos = robot.get_end_effector_position()
            print(f"✓ Home reached: ({home_pos[0]:.1f}, {home_pos[1]:.1f}, {home_pos[2]:.1f})")
        else:
            print("\n✗ Pick and place failed")
            
    except Exception as e:
        print(f"\n✗ Execution error: {e}")
    finally:
        robot.disconnect()
        print("\nDisconnected from robot")

if __name__ == "__main__":
    main()
