#!/usr/bin/env python3
"""
Inverse Kinematics for Prime-Arm
Direct angle mapping: servo 90° = math 90°
"""

import math
import numpy as np

a1 = 16.4
a2 = 10.5
a3 = 14.8
a4 = 18.0


def dh_matrix(theta, alpha, r, d):
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def forward_kinematics(T1, T2, T3, T4):
    """FK using DH parameters. Returns (x, y, z) in cm."""
    DH_PARAMS = [
        [T1, math.radians(90), 0, a1],
        [T2, math.radians(180), a2, 0],
        [T3 + math.radians(90), 0, -a3, 0],
        [T4 + math.radians(90), 0, a4, 0],
    ]

    H = np.eye(4)
    for p in DH_PARAMS:
        H = H @ dh_matrix(*p)

    return H[0, 3], H[1, 3], H[2, 3]


def inverse_kinematics_numeric(x_target, y_target, z_target, initial_guess=None):
    """Numerical IK solver using Jacobian method with multiple initial guesses."""
    if initial_guess is None:
        initial_guesses = [
            [90, 90, 90, 90],
            [90, 120, 60, 90],
            [90, 60, 120, 90],
            [140, 106, 81, 103],
            [45, 90, 90, 90],
            [135, 90, 90, 90],
        ]
        
        best_T = None
        best_error = float('inf')
        
        for guess in initial_guesses:
            T = np.array([math.radians(g) for g in guess])
            T_result, final_error = _solve_ik(x_target, y_target, z_target, T)
            
            if final_error < best_error:
                best_error = final_error
                best_T = T_result
                if final_error < 0.1:
                    break
        
        return best_T
    else:
        T = np.array(initial_guess)
        T_result, _ = _solve_ik(x_target, y_target, z_target, T)
        return T_result


def _solve_ik(x_target, y_target, z_target, T_initial):
    """Internal IK solver for a single initial guess."""
    T = T_initial.copy()

    for i in range(300):
        x, y, z = forward_kinematics(*T)

        error = np.array([x_target - x, y_target - y, z_target - z])
        error_norm = np.linalg.norm(error)

        if error_norm < 1e-3:
            break

        J = np.zeros((3, 4))
        delta = 1e-5

        for j in range(4):
            T_temp = T.copy()
            T_temp[j] += delta
            x1, y1, z1 = forward_kinematics(*T_temp)

            J[:, j] = [(x1 - x)/delta, (y1 - y)/delta, (z1 - z)/delta]

        J_pinv = np.linalg.pinv(J)
        dT = J_pinv @ error
        step_size = min(0.1, error_norm / 10)
        dT = np.clip(dT, -step_size, step_size)

        T += dT
        T = np.clip(T, 0, math.pi)

    x, y, z = forward_kinematics(*T)
    final_error = math.sqrt((x_target - x)**2 + (y_target - y)**2 + (z_target - z)**2)
    
    return T, final_error



def math_angles_to_servo_angles(T1, T2, T3, T4):
    """Convert mathematical joint angles to servo angles (0-180°)."""
    base_servo = math.degrees(T1)
    shoulder_servo = math.degrees(T2)
    elbow_servo = math.degrees(T3)
    wrist_servo = math.degrees(T4)
    
    base_servo = max(0, min(180, base_servo))
    shoulder_servo = max(0, min(180, shoulder_servo))
    elbow_servo = max(0, min(180, elbow_servo))
    wrist_servo = max(0, min(180, wrist_servo))
    
    return {
        'b': round(base_servo),
        's': round(shoulder_servo),
        'e': round(elbow_servo),
        'w': round(wrist_servo)
    }


def servo_angles_to_math_angles(base, shoulder, elbow, wrist):
    """Convert servo angles (0-180°) to mathematical angles in radians."""
    T1 = math.radians(base)
    T2 = math.radians(shoulder)
    T3 = math.radians(elbow)
    T4 = math.radians(wrist)
    
    return T1, T2, T3, T4


def calculate_ik(x, y, z, initial_servo_angles=None):
    """Calculate IK and return servo angles with success status and error."""
    initial_guess = None
    if initial_servo_angles is not None:
        initial_guess = servo_angles_to_math_angles(
            initial_servo_angles['b'],
            initial_servo_angles['s'],
            initial_servo_angles['e'],
            initial_servo_angles['w']
        )
    
    T = inverse_kinematics_numeric(x, y, z, initial_guess)
    servo_angles = math_angles_to_servo_angles(*T)
    
    xf, yf, zf = forward_kinematics(*T)
    error = math.sqrt((x - xf)**2 + (y - yf)**2 + (z - zf)**2)
    success = error < 0.5
    
    return servo_angles, success, error
if __name__ == "__main__":
    x, y, z = -0.88, 0.85, 56.47
    print(f"\nTarget Position: ({x:.2f}, {y:.2f}, {z:.2f}) cm")
    
    servo_angles, success, error = calculate_ik(x, y, z)
    
    print(f"Servo Angles: b={servo_angles['b']}° s={servo_angles['s']}° e={servo_angles['e']}° w={servo_angles['w']}°")
    print(f"Status: {'SUCCESS' if success else 'FAILED'} | Error: {error:.4f} cm")
    
    T = servo_angles_to_math_angles(
        servo_angles['b'], servo_angles['s'], 
        servo_angles['e'], servo_angles['w']
    )
    xf, yf, zf = forward_kinematics(*T)
    print(f"Achieved: ({xf:.2f}, {yf:.2f}, {zf:.2f}) cm")
