#!/usr/bin/env python3
"""
Raspberry Pi Controller for Prime-Arm Robot
Handles serial communication with Arduino with smooth motion control
"""

import serial
import time
import threading
import math
import numpy as np
from forward_kinematics import forward_kinematics, dh_matrix
from inverse_kinematics_servo import calculate_ik


class RobotArmController:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, step_delay=0.015):
        """
        Initialize the robot arm controller
        
        Args:
            port: Serial port (default: /dev/ttyACM0 on Raspberry Pi)
            baudrate: Communication speed (default: 115200)
            step_delay: Delay between 1-degree steps in seconds (default: 0.015)
        """
        self.port = port
        self.baudrate = baudrate
        self.step_delay = step_delay
        self.serial_conn = None
        self.current_angles = {'b': 0, 's': 0, 'e': 0, 'w': 0, 't': 0, 'g': 80}
        self.feedback_lock = threading.Lock()
        self.listening = False
        self.ack_received = threading.Event()
        
        # Forward kinematics parameters (link lengths in cm)
        self.a1 = 16.4
        self.a2 = 10.5
        self.a3 = 14.8
        self.a4 = 18.0
        
    def connect(self):
        """Establish serial connection with Arduino"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)
            print(f"Connected to {self.port}")
            
            # Start feedback listener thread
            self.listening = True
            self.listener_thread = threading.Thread(target=self._listen_feedback, daemon=True)
            self.listener_thread.start()
            
            # Get initial angles from Arduino
            time.sleep(0.5)
            self._request_current_angles()
            time.sleep(0.5)
            
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        self.listening = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected")
    
    def _listen_feedback(self):
        """Background thread to listen for feedback from Arduino"""
        while self.listening and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    
                    if line.startswith('FEEDBACK:'):
                        self._parse_feedback(line)
                    elif line == 'ACK':
                        self.ack_received.set()
            except Exception as e:
                print(f"Error reading feedback: {e}")
            time.sleep(0.001)
    
    def _parse_feedback(self, feedback_line):
        """Parse feedback from Arduino"""
        try:
            data = feedback_line.replace('FEEDBACK:', '')
            parts = data.split(',')
            
            with self.feedback_lock:
                for part in parts:
                    if len(part) >= 2:
                        servo = part[0]
                        angle = int(part[1:])
                        self.current_angles[servo] = angle
        except Exception as e:
            print(f"Error parsing feedback: {e}")
    
    def _request_current_angles(self):
        """Request current angles from Arduino"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(b'GET\n')
    
    def _send_raw_command(self, angles_dict):
        """
        Send raw command to Arduino (internal use)
        
        Args:
            angles_dict: Dictionary with servo angles
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("Error: Not connected to Arduino")
            return False
        
        command_parts = []
        for servo in ['b', 's', 'e', 'w', 't', 'g']:
            if servo in angles_dict:
                command_parts.append(f"{servo}{angles_dict[servo]}")
        
        command = ','.join(command_parts) + '\n'
        
        try:
            self.ack_received.clear()
            self.serial_conn.write(command.encode('utf-8'))
            # Wait for ACK with timeout
            self.ack_received.wait(timeout=0.1)
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def send_command(self, angles_dict, smooth=True, display_progress=True):
        """
        Send movement command with smooth 1-degree stepping
        
        Args:
            angles_dict: Dictionary with servo angles, e.g., {'b': 90, 's': 45, 'e': 90}
            smooth: If True, moves in 1-degree steps. If False, moves directly.
            display_progress: If True, displays angle changes in real-time
        
        Example:
            robot.send_command({'b': 90, 's': 45, 'e': 90, 'w': 60, 't': 45, 'g': 80})
        """
        if not smooth:
            return self._send_raw_command(angles_dict)
        
        current = self.get_current_angles()
        
        target = {}
        for servo, angle in angles_dict.items():
            if servo == 'g':
                target[servo] = max(0, min(80, angle))
            else:
                target[servo] = max(0, min(180, angle))
        
        max_steps = 0
        for servo in target:
            if servo in current:
                max_steps = max(max_steps, abs(target[servo] - current[servo]))
        
        if max_steps == 0:
            if display_progress:
                print("Already at target position")
            return True
        
        if display_progress:
            print(f"\nMoving from {current} to {target}")
            print(f"Maximum steps: {max_steps} degrees\n")
        
        step_angles = current.copy()
        
        for step in range(max_steps):
            command = {}
            
            for servo in target:
                if servo not in step_angles:
                    step_angles[servo] = current.get(servo, 0)
                
                current_val = step_angles[servo]
                target_val = target[servo]
                
                if current_val < target_val:
                    step_angles[servo] = min(current_val + 1, target_val)
                elif current_val > target_val:
                    step_angles[servo] = max(current_val - 1, target_val)
                
                command[servo] = step_angles[servo]
            
            self._send_raw_command(command)
            
            if display_progress:
                progress_str = " | ".join([f"{s.upper()}:{command[s]:3d}°" for s in ['b', 's', 'e', 'w', 't', 'g'] if s in command])
                print(f"\r[Step {step+1:3d}/{max_steps:3d}] {progress_str}", end='', flush=True)
            
            time.sleep(self.step_delay)
        
        if display_progress:
            print("\n\nMovement complete!")
        
        with self.feedback_lock:
            self.current_angles.update(target)
        
        return True
    
    def move_servo(self, servo, angle, smooth=True, display_progress=True):
        """
        Move a single servo to specified angle
        
        Args:
            servo: Servo identifier ('b', 's', 'e', 'w', 't', 'g')
            angle: Target angle (0-180, gripper: 0-80)
            smooth: If True, moves in 1-degree steps
            display_progress: If True, displays angle changes
        """
        return self.send_command({servo: angle}, smooth=smooth, display_progress=display_progress)
    
    def get_current_angles(self):
        """Get current servo angles"""
        with self.feedback_lock:
            return self.current_angles.copy()
    
    def home_position(self, smooth=True, display_progress=True):
        """Move all servos to home position"""
        return self.send_command({
            'b': 0, 
            's': 0, 
            'e': 0, 
            'w': 0, 
            't': 0, 
            'g': 80
        }, smooth=smooth, display_progress=display_progress)
    
    def calculate_fk(self, angles=None):
        """
        Calculate forward kinematics to get end effector position
        
        Args:
            angles: Dictionary with servo angles {'b': 0, 's': 0, 'e': 0, 'w': 0}
                   If None, uses current angles
        
        Returns:
            (x, y, z): End effector position in cm
        """
        if angles is None:
            angles = self.get_current_angles()
        
        # Convert angles to radians
        T1 = math.radians(angles['b'])   # Base
        T2 = math.radians(angles['s'])   # Shoulder
        T3 = math.radians(angles['e'])   # Elbow
        T4 = math.radians(angles['w'])   # Wrist
        
        # Build DH parameters
        dh_params = [
            [T1, math.radians(90), 0, self.a1],
            [T2, math.radians(180), self.a2, 0],
            [T3 + math.radians(90), 0, -self.a3, 0],
            [T4 + math.radians(90), 0, self.a4, 0],
        ]
        
        # Calculate forward kinematics
        x, y, z = forward_kinematics(dh_params)
        return x, y, z
    
    def get_end_effector_position(self):
        """
        Get current end effector position in 3D space
        
        Returns:
            (x, y, z): End effector position in cm
        """
        return self.calculate_fk()
    
    def move_to_position(self, x, y, z, smooth=True, display_progress=True):
        """
        Move end effector to target position using inverse kinematics
        
        Args:
            x, y, z: Target position in cm
            smooth: If True, moves in 1-degree steps
            display_progress: If True, displays movement progress
        
        Returns:
            success: Boolean indicating if IK solution was found and executed
        """
        if display_progress:
            print(f"\n{'='*50}")
            print(f"Moving to position: ({x:.2f}, {y:.2f}, {z:.2f}) cm")
            print(f"{'='*50}")
        
        # Get current angles as initial guess for IK solver
        current = self.get_current_angles()
        initial_guess = {
            'b': current['b'],
            's': current['s'],
            'e': current['e'],
            'w': current['w']
        }
        
        # Solve inverse kinematics
        servo_angles, success, error = calculate_ik(x, y, z, initial_guess)
        
        if display_progress:
            print(f"\nIK Solution:")
            print(f"  Base (b):      {servo_angles['b']:3d}°")
            print(f"  Shoulder (s):  {servo_angles['s']:3d}°")
            print(f"  Elbow (e):     {servo_angles['e']:3d}°")
            print(f"  Wrist (w):     {servo_angles['w']:3d}°")
            print(f"  Position Error: {error:.4f} cm")
            print(f"  Status: {'SUCCESS' if success else 'FAILED'}")
        
        if not success:
            print(f"\nWARNING: IK solution has high error ({error:.4f} cm)")
            print(f"Proceeding anyway...\n")
        
        # Execute movement (keep twist and gripper at current positions)
        move_command = {
            'b': servo_angles['b'],
            's': servo_angles['s'],
            'e': servo_angles['e'],
            'w': servo_angles['w'],
            't': current['t'],
            'g': current['g']
        }
        
        result = self.send_command(move_command, smooth=smooth, display_progress=display_progress)
        
        if display_progress and result:
            # Verify final position
            final_x, final_y, final_z = self.get_end_effector_position()
            print(f"\nFinal Position Verification:")
            print(f"  Target:   ({x:7.2f}, {y:7.2f}, {z:7.2f}) cm")
            print(f"  Achieved: ({final_x:7.2f}, {final_y:7.2f}, {final_z:7.2f}) cm")
            actual_error = math.sqrt((x-final_x)**2 + (y-final_y)**2 + (z-final_z)**2)
            print(f"  Error:    {actual_error:7.4f} cm")
            print(f"{'='*50}\n")
        
        return success
    
    def print_position(self):
        """Print current joint angles and end effector position"""
        angles = self.get_current_angles()
        x, y, z = self.calculate_fk(angles)
        
        print(f"\n{'='*50}")
        print(f"Joint Angles:")
        print(f"  Base (b):      {angles['b']:3d}°")
        print(f"  Shoulder (s):  {angles['s']:3d}°")
        print(f"  Elbow (e):     {angles['e']:3d}°")
        print(f"  Wrist (w):     {angles['w']:3d}°")
        print(f"  Twist (t):     {angles['t']:3d}°")
        print(f"  Gripper (g):   {angles['g']:3d}°")
        print(f"\nEnd Effector Position:")
        print(f"  X: {x:7.2f} cm")
        print(f"  Y: {y:7.2f} cm")
        print(f"  Z: {z:7.2f} cm")
        print(f"{'='*50}\n")



def main():
    """Example usage"""
    robot = RobotArmController(port='/dev/ttyACM0', baudrate=115200, step_delay=0.015)

    if not robot.connect():
        print("Failed to connect. Check the port and try again.")
        return
    
    try:
        print("\nRobot connected. Current angles:")
        print(robot.get_current_angles())
        print("\nAdd your robot control code here.")
        print("Press Ctrl+C to disconnect.\n")
        
        # Keep running until interrupted
        while True:
            time.sleep(0.1)
        
    except KeyboardInterrupt:
        print("\nDisconnecting...")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
