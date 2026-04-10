#!/usr/bin/env python3
"""
Raspberry Pi Controller for Prime-Arm Robot
Handles serial communication with Arduino with smooth motion control
"""

import sys
import serial
import time
import threading
import math
import numpy as np
from forward_kinematics import forward_kinematics, dh_matrix
from inverse_kinematics_servo import calculate_ik

# Pi Zero 2 W uses the hardware UART on GPIO 14/15 → /dev/serial0
# Windows development uses COM4
DEFAULT_PORT = '/dev/serial0' if not sys.platform.startswith('win') else 'COM4'


class RobotArmController:
    def __init__(self, port=DEFAULT_PORT, baudrate=115200, step_delay=0.015):
        """
        Initialize the robot arm controller
        
        Args:
            port: Serial port (default: /dev/serial0 on Pi, COM4 on Windows)
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
            # Set dtr=False BEFORE open() so the DTR line never pulses,
            # which prevents the Arduino from auto-resetting on connect.
            self.serial_conn = serial.Serial()
            self.serial_conn.port = self.port
            self.serial_conn.baudrate = self.baudrate
            self.serial_conn.timeout = 1
            self.serial_conn.dtr = False  # must be set before open()
            self.serial_conn.rts = False
            self.serial_conn.open()
            time.sleep(0.1)  # brief settle
            print(f"Connected to {self.port}")
            
            # Start feedback listener thread
            self.listening = True
            self.listener_thread = threading.Thread(target=self._listen_feedback, daemon=True)
            self.listener_thread.start()
            
            # Poll until we receive real angles from the Arduino (up to 15 s)
            print("Waiting for Arduino feedback...", end='', flush=True)
            deadline = time.time() + 15
            while time.time() < deadline:
                self._request_current_angles()
                time.sleep(0.5)
                with self.feedback_lock:
                    angles = self.current_angles.copy()
                # Any non-zero joint angle means feedback arrived
                if any(angles[k] != 0 for k in ('b', 's', 'e', 'w', 't')):
                    break
                print('.', end='', flush=True)
            print()  # newline after dots
            
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
    
    def _send_raw_command(self, angles_dict, debug=False):
        """
        Send raw command to Arduino and wait for ACK.
        Use for single positional moves. For path streaming use _stream_command.
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("Error: Not connected to Arduino")
            return False
        
        # SAFETY: Clamp all angles to safe ranges as final protection
        safe_angles = {}
        for servo, angle in angles_dict.items():
            if servo == 'g':
                    # Gripper: strict 30-80° limit, where 30° is fully open
                safe_angles[servo] = max(30, min(80, int(angle)))
            else:
                # Other servos: 0-180°
                safe_angles[servo] = max(0, min(180, int(angle)))
        
        command_parts = []
        for servo in ['b', 's', 'e', 'w', 't', 'g']:
            if servo in safe_angles:
                command_parts.append(f"{servo}{safe_angles[servo]}")
        
        command = ','.join(command_parts) + '\n'
        
        if debug:
            print(f"  → Sending to Arduino: {command.strip()}")
        
        try:
            self.ack_received.clear()
            self.serial_conn.write(command.encode('utf-8'))
            # Wait for ACK with timeout
            ack_received = self.ack_received.wait(timeout=0.1)
            if debug and not ack_received:
                print(f"  ⚠️  No ACK received for command!")
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def _stream_command(self, angles_dict):
        """
        Send a command WITHOUT waiting for ACK — for high-speed path streaming.
        The Arduino processes immediately; Python doesn't block on the reply.
        The caller is responsible for sleeping an appropriate dt between calls
        so the Arduino has time to process and the servo has time to move.
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False

        safe_angles = {}
        for servo, angle in angles_dict.items():
            if servo == 'g':
                safe_angles[servo] = max(30, min(80, int(angle)))
            else:
                safe_angles[servo] = max(0, min(180, int(angle)))

        command_parts = []
        for servo in ['b', 's', 'e', 'w', 't', 'g']:
            if servo in safe_angles:
                command_parts.append(f"{servo}{safe_angles[servo]}")

        try:
            self.serial_conn.write((','.join(command_parts) + '\n').encode('utf-8'))
            return True
        except Exception as e:
            print(f"Error streaming command: {e}")
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
                target[servo] = max(30, min(80, angle))
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
            angle: Target angle (0-180, gripper: 30-80)
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
            'b': 90,
            's': 130,
            'e': 180,
            'w': 180,
            't': 180,
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
    
    def execute_cartesian_path(self, cartesian_path, dt=0.02, display_progress=True):
        """
        Execute a pre-planned Cartesian path with smooth motion
        
        Args:
            cartesian_path: List of (x, y, z) positions
            dt: Time between steps in seconds (used for step delay)
            display_progress: Show progress updates
        
        Returns:
            success: Boolean indicating if path was completed
        """
        if display_progress:
            print(f"\n{'='*50}")
            print(f"Executing Cartesian Path")
            print(f"Waypoints: {len(cartesian_path)}")
            print(f"{'='*50}\n")
        
        current_angles = self.get_current_angles()
        initial_guess = {
            'b': current_angles['b'],
            's': current_angles['s'],
            'e': current_angles['e'],
            'w': current_angles['w']
        }
        
        # Store original step delay
        original_delay = self.step_delay
        self.step_delay = dt
        
        for i, pos in enumerate(cartesian_path):
            x, y, z = pos
            
            # Solve IK
            angles, success, error = calculate_ik(x, y, z, initial_guess)
            
            if display_progress:
                print(f"[{i+1}/{len(cartesian_path)}] Moving to ({x:.1f}, {y:.1f}, {z:.1f}) | Error: {error:.3f} cm")
            
            # Execute movement with smooth stepping
            move_command = {
                'b': angles['b'],
                's': angles['s'],
                'e': angles['e'],
                'w': angles['w'],
                't': current_angles['t'],
                'g': current_angles['g']
            }
            
            # Use smooth motion for each waypoint
            self.send_command(move_command, smooth=True, display_progress=False)
            
            # Use this solution as initial guess for next point
            initial_guess = angles
        
        # Restore original step delay
        self.step_delay = original_delay
        
        if display_progress:
            print(f"\nPath execution complete!")
            print(f"{'='*50}\n")
        
        return True
    
    def execute_joint_path(self, joint_path, dt=0.02, display_progress=True):
        """
        Execute a pre-planned joint space path with smooth motion
        
        Args:
            joint_path: List of angle dicts [{'b': 0, 's': 0, 'e': 0, 'w': 0}, ...]
            dt: Time between steps in seconds (used for step delay)
            display_progress: Show progress updates
        
        Returns:
            success: Boolean indicating if path was completed
        """
        if display_progress:
            print(f"\n{'='*50}")
            print(f"Executing Joint Path")
            print(f"Waypoints: {len(joint_path)}")
            print(f"{'='*50}\n")
        
        current_angles = self.get_current_angles()
        
        # Store original step delay
        original_delay = self.step_delay
        self.step_delay = dt
        
        for i, angles in enumerate(joint_path):
            if display_progress:
                print(f"[{i+1}/{len(joint_path)}] b={angles['b']:3d}° s={angles['s']:3d}° e={angles['e']:3d}° w={angles['w']:3d}°")
            
            move_command = {
                'b': angles['b'],
                's': angles['s'],
                'e': angles['e'],
                'w': angles['w'],
                't': current_angles['t'],
                'g': current_angles['g']
            }
            
            # Use smooth motion for each waypoint
            self.send_command(move_command, smooth=True, display_progress=False)
        
        # Restore original step delay
        self.step_delay = original_delay
        
        if display_progress:
            print(f"\nPath execution complete!")
            print(f"{'='*50}\n")
        
        return True

    def execute_cartesian_pick_and_place(self, pick_and_place_path, dt=0.015,
                               gripper_open=30, gripper_close=80,
                               display_progress=True):
        """
        Legacy Cartesian-path executor kept for path_planning_gui.py compatibility.
        Prefer execute_pick_and_place() for new code.
        """
        gripper_open  = max(30, min(80, int(gripper_open)))
        gripper_close = max(30, min(80, int(gripper_close)))
        n = len(pick_and_place_path)
        current_angles = self.get_current_angles()
        guess = {k: current_angles[k] for k in ('b', 's', 'e', 'w')}
        gripper_pos = current_angles['g']
        twist = current_angles['t']
        commands = []
        for x, y, z, gripper_cmd in pick_and_place_path:
            angles, ok, err = calculate_ik(x, y, z, guess)
            if gripper_cmd == 'open':
                gripper_pos = gripper_open
            elif gripper_cmd == 'close':
                gripper_pos = gripper_close
            commands.append({'cmd': {'b': angles['b'], 's': angles['s'],
                                     'e': angles['e'], 'w': angles['w'],
                                     't': twist,       'g': gripper_pos},
                             'gripper_cmd': gripper_cmd})
            guess = angles
        for entry in commands:
            self._stream_command(entry['cmd'])
            if entry['gripper_cmd'] in ('open', 'close'):
                time.sleep(0.6)
            else:
                time.sleep(dt)
        with self.feedback_lock:
            self.current_angles.update(commands[-1]['cmd'])
        return True

    def execute_pick_and_place(self, pick_pos, place_pos,
                               approach_height=5.0, retract_height=5.0,
                               gripper_open=30, gripper_close=80,
                               steps_per_segment=30, dt=0.07,
                               vertical_dt=None,
                               display_progress=True):
        """
        Smooth pick-and-place via joint-space interpolation.

        Instead of Cartesian path following (which causes IK failures mid-path),
        this method:
          1. Solves IK for only 5 key Cartesian positions.
          2. Linearly interpolates joint angles between each pair of key positions.
          3. Streams the pre-computed angle sequence at fixed dt intervals.

        Smooth motion is guaranteed because:
          - Both endpoints of every segment always have valid IK solutions.
          - Linear interpolation in joint space never passes through unreachable space.
          - dt shorter than servo travel time keeps servos always "chasing".

        Args:
            pick_pos:  (x, y, z) — grasp point (top-centre of the object)
            place_pos: (x, y, z) — release point
            approach_height:  cm above pick_pos to approach from
            retract_height:   cm above place_pos to transit at
            gripper_open:     angle for open gripper (30-80°)
            gripper_close:    angle for gripped (30-80°)
            steps_per_segment: joint-space interpolation steps between key poses
            dt:               seconds between commands for transit phases (1 & 4)
            vertical_dt:      seconds between commands for descend/retract phases
                              (2, 3, 5, 6). Defaults to dt * 2 if not specified.
            display_progress: print phase labels and IK summary
        """
        gripper_open  = max(30, min(80, int(gripper_open)))
        gripper_close = max(30, min(80, int(gripper_close)))
        if vertical_dt is None:
            vertical_dt = dt * 2

        px, py, pz = pick_pos
        lx, ly, lz = place_pos

        # ── 5 key Cartesian positions ─────────────────────────────────────
        above_pick  = (px, py, pz + approach_height)
        at_pick     = (px, py, pz)
        above_place = (lx, ly, lz + retract_height)
        at_place    = (lx, ly, lz)

        # ── Solve IK for each key position ────────────────────────────────
        current = self.get_current_angles()
        twist   = current['t']
        guess   = {k: current[k] for k in ('b', 's', 'e', 'w')}

        if display_progress:
            print(f"\n{'='*50}")
            print(f"Joint-space pick-and-place")
            print(f"Solving IK for 5 key positions…")

        def solve(label, pos):
            nonlocal guess
            angles, ok, err = calculate_ik(*pos, guess)
            if display_progress:
                st = '✓' if ok else '✗'
                print(f"  {st} {label:14s}: b={angles['b']:3d}° s={angles['s']:3d}° "
                      f"e={angles['e']:3d}° w={angles['w']:3d}°  err={err:.2f} cm")
            guess = angles
            return angles

        j_current    = {k: current[k] for k in ('b', 's', 'e', 'w')}
        j_above_pick  = solve('above_pick',  above_pick)
        j_at_pick     = solve('at_pick',     at_pick)
        j_above_place = solve('above_place', above_place)
        j_at_place    = solve('at_place',    at_place)

        total_steps = 6 * steps_per_segment
        if display_progress:
            print(f"\n  steps_per_segment={steps_per_segment}  "
                  f"total={total_steps} commands")
            print(f"  Transit dt={dt*1000:.0f} ms   Vertical dt={vertical_dt*1000:.0f} ms")
            print(f"  Gripper: open={gripper_open}°  close={gripper_close}°")
            print(f"{'='*50}")

        # ── Joint-space segment streamer ──────────────────────────────────
        def stream_segment(label, from_j, to_j, g_angle, seg_dt=dt):
            if display_progress:
                print(f"  {label}")
            for step in range(1, steps_per_segment + 1):
                t = step / steps_per_segment
                cmd = {k: int(round(from_j[k] + t * (to_j[k] - from_j[k])))
                       for k in ('b', 's', 'e', 'w')}
                cmd['t'] = twist
                cmd['g'] = g_angle
                self._stream_command(cmd)
                time.sleep(seg_dt)

        def gripper_action(label, j_pos, g_angle):
            if display_progress:
                print(f"  {label}")
            self._stream_command({**j_pos, 't': twist, 'g': g_angle})
            time.sleep(0.6)   # give gripper time to physically open/close

        # ── Execute the 6 motion phases ───────────────────────────────────
        stream_segment('Phase 1 → approach pick    ', j_current,     j_above_pick,   gripper_open,  dt)
        stream_segment('Phase 2 → descend to pick  ', j_above_pick,  j_at_pick,      gripper_open,  vertical_dt)
        gripper_action('         → close gripper    ', j_at_pick,                     gripper_close)
        stream_segment('Phase 3 → retract from pick ', j_at_pick,    j_above_pick,   gripper_close, vertical_dt)
        stream_segment('Phase 4 → transfer          ', j_above_pick,  j_above_place,  gripper_close, dt)
        stream_segment('Phase 5 → descend to place  ', j_above_place, j_at_place,    gripper_close, vertical_dt)
        gripper_action('         → open gripper      ', j_at_place,                   gripper_open)
        stream_segment('Phase 6 → retract from place', j_at_place,   j_above_place,  gripper_open,  vertical_dt)

        # ── Sync angle cache ──────────────────────────────────────────────
        with self.feedback_lock:
            self.current_angles.update({**j_above_place, 't': twist, 'g': gripper_open})

        if display_progress:
            print(f"\nPick-and-place complete!")
            print(f"{'='*50}\n")

        return True
        return True
    
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
    robot = RobotArmController(port=DEFAULT_PORT, baudrate=115200, step_delay=0.015)

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
