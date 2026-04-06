#!/usr/bin/env python3
"""
Path Planning for Prime-Arm Robot
Supports Cartesian and joint space path planning
"""

import math
import numpy as np
from inverse_kinematics_servo import calculate_ik, forward_kinematics, servo_angles_to_math_angles


class PathPlanner:
    """Path planning for robot arm"""
    
    def __init__(self):
        self.workspace_limits = {
            'x': (-43.3, 43.3),
            'y': (-43.3, 43.3),
            'z': (0, 60)
        }
    
    def plan_cartesian_line(self, start_pos, end_pos, num_points=20):
        """
        Plan straight line path in Cartesian space
        
        Args:
            start_pos: (x, y, z) starting position in cm
            end_pos: (x, y, z) ending position in cm
            num_points: Number of waypoints along path
        
        Returns:
            List of (x, y, z) positions
        """
        start = np.array(start_pos)
        end = np.array(end_pos)
        
        # Linear interpolation
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            point = start + t * (end - start)
            path.append(tuple(point))
        
        return path
    
    def plan_cartesian_arc(self, start_pos, end_pos, arc_height, num_points=20):
        """
        Plan arc path in Cartesian space (useful for pick-and-place)
        
        Args:
            start_pos: (x, y, z) starting position
            end_pos: (x, y, z) ending position
            arc_height: Additional height at midpoint (cm)
            num_points: Number of waypoints
        
        Returns:
            List of (x, y, z) positions
        """
        start = np.array(start_pos)
        end = np.array(end_pos)
        
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            # Linear interpolation in XY
            point = start + t * (end - start)
            # Parabolic arc in Z
            z_offset = 4 * arc_height * t * (1 - t)  # Parabola peaks at t=0.5
            point[2] += z_offset
            path.append(tuple(point))
        
        return path
    
    def plan_joint_interpolation(self, start_angles, end_angles, num_points=20):
        """
        Plan path with linear interpolation in joint space
        
        Args:
            start_angles: Dict {'b': angle, 's': angle, 'e': angle, 'w': angle}
            end_angles: Dict with same format
            num_points: Number of waypoints
        
        Returns:
            List of angle dicts
        """
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            angles = {}
            for key in ['b', 's', 'e', 'w']:
                angles[key] = int(start_angles[key] + t * (end_angles[key] - start_angles[key]))
            path.append(angles)
        
        return path
    
    def plan_waypoints_cartesian(self, waypoints, num_points_per_segment=15):
        """
        Plan path through multiple Cartesian waypoints
        
        Args:
            waypoints: List of (x, y, z) positions
            num_points_per_segment: Points between each waypoint pair
        
        Returns:
            List of (x, y, z) positions
        """
        if len(waypoints) < 2:
            return waypoints
        
        full_path = []
        for i in range(len(waypoints) - 1):
            segment = self.plan_cartesian_line(
                waypoints[i], 
                waypoints[i + 1], 
                num_points_per_segment
            )
            # Avoid duplicating waypoints
            if i < len(waypoints) - 2:
                segment = segment[:-1]
            full_path.extend(segment)
        
        return full_path
    
    def cartesian_path_to_joint_path(self, cartesian_path, current_angles=None):
        """
        Convert Cartesian path to joint angles using IK
        
        Args:
            cartesian_path: List of (x, y, z) positions
            current_angles: Starting joint angles for IK initial guess
        
        Returns:
            joint_path: List of angle dicts
            success_flags: List of booleans indicating IK success
            errors: List of position errors
        """
        joint_path = []
        success_flags = []
        errors = []
        
        initial_guess = current_angles
        
        for pos in cartesian_path:
            x, y, z = pos
            angles, success, error = calculate_ik(x, y, z, initial_guess)
            
            joint_path.append(angles)
            success_flags.append(success)
            errors.append(error)
            
            # Use previous solution as initial guess for next
            initial_guess = angles
        
        return joint_path, success_flags, errors
    
    def check_workspace_limits(self, position):
        """
        Check if position is within workspace
        
        Args:
            position: (x, y, z) position in cm
        
        Returns:
            Boolean indicating if position is valid
        """
        x, y, z = position
        
        if not (self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1]):
            return False
        if not (self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1]):
            return False
        if not (self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1]):
            return False
        
        return True
    
    def smooth_joint_path(self, joint_path, window_size=3):
        """
        Apply moving average smoothing to joint path
        
        Args:
            joint_path: List of angle dicts
            window_size: Smoothing window size (odd number)
        
        Returns:
            Smoothed joint path
        """
        if len(joint_path) < window_size:
            return joint_path
        
        smoothed = []
        half_window = window_size // 2
        
        for i in range(len(joint_path)):
            start_idx = max(0, i - half_window)
            end_idx = min(len(joint_path), i + half_window + 1)
            
            angles = {}
            for key in ['b', 's', 'e', 'w']:
                values = [joint_path[j][key] for j in range(start_idx, end_idx)]
                angles[key] = int(sum(values) / len(values))
            
            smoothed.append(angles)
        
        return smoothed
    
    def plan_cartesian_line_cubic(self, start_pos, end_pos, num_points=20, 
                                   start_vel=(0, 0, 0), end_vel=(0, 0, 0)):
        """
        Plan smooth path with cubic polynomial interpolation
        Ensures continuous position and velocity
        
        Args:
            start_pos: (x, y, z) starting position in cm
            end_pos: (x, y, z) ending position in cm
            num_points: Number of waypoints along path
            start_vel: (vx, vy, vz) initial velocity (cm/s), default zero
            end_vel: (vx, vy, vz) final velocity (cm/s), default zero
        
        Returns:
            List of (x, y, z) positions
        """
        start = np.array(start_pos)
        end = np.array(end_pos)
        v0 = np.array(start_vel)
        v1 = np.array(end_vel)
        
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Cubic polynomial: p(t) = a0 + a1*t + a2*t² + a3*t³
            # Boundary conditions:
            #   p(0) = start, p(1) = end
            #   p'(0) = v0, p'(1) = v1
            # Solution:
            #   a0 = start
            #   a1 = v0
            #   a2 = 3*(end - start) - 2*v0 - v1
            #   a3 = 2*(start - end) + v0 + v1
            
            a0 = start
            a1 = v0
            a2 = 3 * (end - start) - 2 * v0 - v1
            a3 = 2 * (start - end) + v0 + v1
            
            point = a0 + a1*t + a2*(t**2) + a3*(t**3)
            path.append(tuple(point))
        
        return path
    
    def plan_cartesian_line_quintic(self, start_pos, end_pos, num_points=20,
                                     start_vel=(0, 0, 0), end_vel=(0, 0, 0),
                                     start_acc=(0, 0, 0), end_acc=(0, 0, 0)):
        """
        Plan smooth path with quintic (5th order) polynomial interpolation
        Ensures continuous position, velocity, and acceleration
        
        Args:
            start_pos: (x, y, z) starting position in cm
            end_pos: (x, y, z) ending position in cm
            num_points: Number of waypoints along path
            start_vel: (vx, vy, vz) initial velocity (cm/s), default zero
            end_vel: (vx, vy, vz) final velocity (cm/s), default zero
            start_acc: (ax, ay, az) initial acceleration (cm/s²), default zero
            end_acc: (ax, ay, az) final acceleration (cm/s²), default zero
        
        Returns:
            List of (x, y, z) positions
        """
        start = np.array(start_pos)
        end = np.array(end_pos)
        v0 = np.array(start_vel)
        v1 = np.array(end_vel)
        a0 = np.array(start_acc)
        a1 = np.array(end_acc)
        
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Quintic polynomial: p(t) = c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵
            # Boundary conditions:
            #   p(0) = start, p(1) = end
            #   p'(0) = v0, p'(1) = v1
            #   p''(0) = a0, p''(1) = a1
            # Solution:
            c0 = start
            c1 = v0
            c2 = 0.5 * a0
            c3 = 10*end - 10*start - 6*v0 - 4*v1 - 1.5*a0 + 0.5*a1
            c4 = -15*end + 15*start + 8*v0 + 7*v1 + 1.5*a0 - a1
            c5 = 6*end - 6*start - 3*v0 - 3*v1 - 0.5*a0 + 0.5*a1
            
            point = c0 + c1*t + c2*(t**2) + c3*(t**3) + c4*(t**4) + c5*(t**5)
            path.append(tuple(point))
        
        return path
    
    def plan_waypoints_cubic_spline(self, waypoints, num_points_per_segment=15, 
                                     smooth_velocity=True):
        """
        Plan smooth path through waypoints using cubic spline interpolation
        
        Args:
            waypoints: List of (x, y, z) positions
            num_points_per_segment: Points between each waypoint pair
            smooth_velocity: If True, ensures velocity continuity at waypoints
        
        Returns:
            List of (x, y, z) positions
        """
        if len(waypoints) < 2:
            return waypoints
        
        full_path = []
        
        for i in range(len(waypoints) - 1):
            # Calculate velocities at waypoints for continuity
            if smooth_velocity and i > 0:
                # Use central difference for interior waypoints
                v_start = np.array(waypoints[i]) - np.array(waypoints[i-1])
            else:
                v_start = (0, 0, 0)
            
            if smooth_velocity and i < len(waypoints) - 2:
                v_end = np.array(waypoints[i+2]) - np.array(waypoints[i+1])
            else:
                v_end = (0, 0, 0)
            
            segment = self.plan_cartesian_line_cubic(
                waypoints[i], 
                waypoints[i + 1],
                num_points_per_segment,
                start_vel=v_start,
                end_vel=v_end
            )
            
            # Avoid duplicating waypoints
            if i < len(waypoints) - 2:
                segment = segment[:-1]
            full_path.extend(segment)
        
        return full_path
    
    def compute_path_length(self, cartesian_path):
        """
        Compute total path length in Cartesian space
        
        Args:
            cartesian_path: List of (x, y, z) positions
        
        Returns:
            Total path length in cm
        """
        total_length = 0
        for i in range(len(cartesian_path) - 1):
            p1 = np.array(cartesian_path[i])
            p2 = np.array(cartesian_path[i + 1])
            total_length += np.linalg.norm(p2 - p1)
        
        return total_length
    
    def compute_joint_path_length(self, joint_path):
        """
        Compute total joint motion (sum of all joint movements)
        
        Args:
            joint_path: List of angle dicts
        
        Returns:
            Total joint motion in degrees
        """
        total_motion = 0
        for i in range(len(joint_path) - 1):
            for key in ['b', 's', 'e', 'w']:
                total_motion += abs(joint_path[i + 1][key] - joint_path[i][key])
        
        return total_motion
    
    def plan_pick_and_place(self, pick_pos, place_pos, current_pos=None,
                           approach_height=10, retract_height=10, 
                           num_points_approach=5, num_points_transfer=15,
                           interpolation='linear'):
        """
        Plan complete pick-and-place operation with all 7 phases:
        1. Move to Pre-Pick/Approach Pose
        2. Descend to Pick (Grasp)
        3. Close Gripper
        4. Retract/Lift
        5. Transfer to Pre-Place/Approach Pose
        6. Descend and Place
        7. Retract/Return
        
        Args:
            pick_pos: (x, y, z) pick position in cm
            place_pos: (x, y, z) place position in cm
            current_pos: (x, y, z) current end effector position (optional)
                        If None, path starts at approach position
            approach_height: Height above pick/place for approach (cm)
            retract_height: Height to retract after pick/place (cm)
            num_points_approach: Points for vertical approach/retract
            num_points_transfer: Points for transfer between pick and place
            interpolation: 'linear', 'cubic', or 'quintic' for smooth trajectories
        
        Returns:
            phases: Dict with phase names and their paths
            full_path: Complete path as list of (x, y, z, gripper_state)
        """
        pick_x, pick_y, pick_z = pick_pos
        place_x, place_y, place_z = place_pos
        
        # Define all waypoints
        above_pick = (pick_x, pick_y, pick_z + approach_height)
        at_pick = (pick_x, pick_y, pick_z)
        above_pick_retract = (pick_x, pick_y, pick_z + retract_height)
        
        above_place = (place_x, place_y, place_z + approach_height)
        at_place = (place_x, place_y, place_z)
        above_place_retract = (place_x, place_y, place_z + retract_height)
        
        phases = {}
        
        # Select interpolation function
        if interpolation == 'cubic':
            plan_func = self.plan_cartesian_line_cubic
        elif interpolation == 'quintic':
            plan_func = self.plan_cartesian_line_quintic
        else:  # linear
            plan_func = self.plan_cartesian_line
        
        # Phase 1: Move to Pre-Pick/Approach Pose (from current position)
        if current_pos is not None:
            phases['approach_to_pick'] = plan_func(
                current_pos, above_pick, num_points_transfer
            )
        else:
            # Start at approach position if no current position given
            phases['approach_to_pick'] = [above_pick]
        
        # Phase 2: Descend to Pick (Grasp) - use smooth interpolation
        phases['descend_to_pick'] = plan_func(
            above_pick, at_pick, num_points_approach
        )
        
        # Phase 3: Close Gripper
        phases['close_gripper'] = [at_pick]
        
        # Phase 4: Retract/Lift with object - use smooth interpolation
        phases['retract_pick'] = plan_func(
            at_pick, above_pick_retract, num_points_approach
        )
        
        # Phase 5: Transfer to Pre-Place/Approach Pose (with arc for clearance)
        phases['transfer'] = self.plan_cartesian_arc(
            above_pick_retract, above_place, 
            arc_height=max(retract_height, approach_height), 
            num_points=num_points_transfer
        )
        
        # Phase 6: Descend to Place - use smooth interpolation
        phases['descend_to_place'] = plan_func(
            above_place, at_place, num_points_approach
        )
        
        # Phase 7: Open Gripper
        phases['open_gripper'] = [at_place]
        
        # Phase 8: Retract/Return after place - use smooth interpolation
        phases['retract_place'] = plan_func(
            at_place, above_place_retract, num_points_approach
        )
        
        # Build full path with gripper states
        # Format: (x, y, z, gripper_command)
        # gripper_command: None (no change), 'open', 'close'
        full_path = []
        
        # Phase 1: Approach to pick (gripper OPEN to prepare for grasp)
        for pos in phases['approach_to_pick']:
            full_path.append((*pos, 'open'))
        
        # Phase 2: Descend to pick (gripper still OPEN)
        for pos in phases['descend_to_pick']:
            full_path.append((*pos, 'open'))
        
        # Phase 3: Close gripper to grasp object
        full_path.append((*phases['close_gripper'][0], 'close'))
        
        # Phase 4: Retract with object (gripper CLOSED)
        for pos in phases['retract_pick']:
            full_path.append((*pos, 'close'))
        
        # Phase 5: Transfer (gripper stays CLOSED holding object)
        for pos in phases['transfer']:
            full_path.append((*pos, 'close'))
        
        # Phase 6: Descend to place (gripper still CLOSED)
        for pos in phases['descend_to_place']:
            full_path.append((*pos, 'close'))
        
        # Phase 7: Open gripper
        full_path.append((*phases['open_gripper'][0], 'open'))
        
        # Phase 8: Retract (gripper stays OPEN)
        for pos in phases['retract_place']:
            full_path.append((*pos, 'open'))
        
        return phases, full_path


class TrajectoryPlanner:
    """Time-parameterized trajectory planning"""
    
    def __init__(self, max_velocity=30, max_acceleration=50):
        """
        Initialize trajectory planner
        
        Args:
            max_velocity: Maximum velocity in deg/s
            max_acceleration: Maximum acceleration in deg/s²
        """
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
    
    def plan_trapezoidal_profile(self, start_angle, end_angle, dt=0.02):
        """
        Plan smooth trapezoidal velocity profile
        
        Args:
            start_angle: Starting angle (degrees)
            end_angle: Ending angle (degrees)
            dt: Time step in seconds
        
        Returns:
            times: List of time stamps
            positions: List of positions
            velocities: List of velocities
        """
        distance = abs(end_angle - start_angle)
        direction = 1 if end_angle > start_angle else -1
        
        # Time to accelerate to max velocity
        t_accel = self.max_velocity / self.max_acceleration
        # Distance during acceleration
        d_accel = 0.5 * self.max_acceleration * t_accel**2
        
        # Check if we can reach max velocity
        if 2 * d_accel >= distance:
            # Triangular profile (no constant velocity phase)
            t_accel = math.sqrt(distance / self.max_acceleration)
            d_accel = distance / 2
            t_constant = 0
        else:
            # Trapezoidal profile
            t_constant = (distance - 2 * d_accel) / self.max_velocity
        
        t_total = 2 * t_accel + t_constant
        
        # Generate trajectory
        times = []
        positions = []
        velocities = []
        
        t = 0
        while t <= t_total:
            if t <= t_accel:
                # Acceleration phase
                pos = start_angle + direction * (0.5 * self.max_acceleration * t**2)
                vel = direction * self.max_acceleration * t
            elif t <= t_accel + t_constant:
                # Constant velocity phase
                pos = start_angle + direction * (d_accel + self.max_velocity * (t - t_accel))
                vel = direction * self.max_velocity
            else:
                # Deceleration phase
                t_decel = t - t_accel - t_constant
                pos = end_angle - direction * (0.5 * self.max_acceleration * (t_accel - t_decel)**2)
                vel = direction * self.max_velocity * (1 - t_decel / t_accel)
            
            times.append(t)
            positions.append(pos)
            velocities.append(vel)
            t += dt
        
        return times, positions, velocities


# Convenience functions
def plan_straight_line(start_xyz, end_xyz, num_points=20):
    """Quick function to plan straight line"""
    planner = PathPlanner()
    return planner.plan_cartesian_line(start_xyz, end_xyz, num_points)


def plan_pick_and_place(pick_pos, place_pos, arc_height=10, num_points=30):
    """
    Plan pick-and-place trajectory with arc
    
    Args:
        pick_pos: (x, y, z) pick position
        place_pos: (x, y, z) place position
        arc_height: Height of arc above straight line
        num_points: Number of waypoints
    
    Returns:
        Cartesian path
    """
    planner = PathPlanner()
    return planner.plan_cartesian_arc(pick_pos, place_pos, arc_height, num_points)


if __name__ == "__main__":
    print("Path Planning Module")
    print("=" * 60)
    
    planner = PathPlanner()
    
    # Test 1: Straight line path
    print("\nTest 1: Straight Line Cartesian Path")
    start = (10, 15, 40)
    end = (20, 25, 50)
    path = planner.plan_cartesian_line(start, end, num_points=10)
    print(f"Start: {start}")
    print(f"End: {end}")
    print(f"Waypoints: {len(path)}")
    print(f"Path length: {planner.compute_path_length(path):.2f} cm")
    
    # Test 2: Arc path
    print("\nTest 2: Arc Path")
    arc_path = planner.plan_cartesian_arc(start, end, arc_height=10, num_points=10)
    print(f"Arc height: 10 cm")
    print(f"Path length: {planner.compute_path_length(arc_path):.2f} cm")
    
    # Test 3: Joint interpolation
    print("\nTest 3: Joint Space Interpolation")
    start_angles = {'b': 0, 's': 0, 'e': 0, 'w': 0}
    end_angles = {'b': 90, 's': 90, 'e': 90, 'w': 90}
    joint_path = planner.plan_joint_interpolation(start_angles, end_angles, 10)
    print(f"Start angles: {start_angles}")
    print(f"End angles: {end_angles}")
    print(f"Total joint motion: {planner.compute_joint_path_length(joint_path):.1f}°")
    
    # Test 4: Waypoints
    print("\nTest 4: Multi-Waypoint Path")
    waypoints = [(10, 15, 40), (15, 20, 45), (20, 25, 50), (25, 20, 45)]
    multi_path = planner.plan_waypoints_cartesian(waypoints, num_points_per_segment=5)
    print(f"Waypoints: {len(waypoints)}")
    print(f"Total path points: {len(multi_path)}")
    print(f"Path length: {planner.compute_path_length(multi_path):.2f} cm")
    
    print("\n" + "=" * 60)
    print("Path planning tests complete!")
