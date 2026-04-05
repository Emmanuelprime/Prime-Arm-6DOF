#!/usr/bin/env python3
"""
3D Visualization for Prime-Arm Robot
Coordinate system: +X right, +Y into screen, +Z up
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import math
from inverse_kinematics_servo import forward_kinematics, servo_angles_to_math_angles

# Link lengths
a1 = 16.4
a2 = 10.5
a3 = 14.8
a4 = 18.0


class RobotVisualizer:
    def __init__(self, figsize=(10, 8)):
        """Initialize 3D visualizer"""
        self.fig = plt.figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Robot arm lines
        self.arm_line, = self.ax.plot([], [], [], 'o-', linewidth=3, 
                                      markersize=8, color='blue', label='Robot Arm')
        self.end_effector, = self.ax.plot([], [], [], 'o', markersize=12, 
                                          color='red', label='End Effector')
        
        # Joint markers
        self.joints, = self.ax.plot([], [], [], 'o', markersize=10, 
                                    color='orange', label='Joints')
        
        # Setup plot
        self.setup_plot()
        
        # Current angles
        self.current_angles = {'b': 0, 's': 0, 'e': 0, 'w': 0}
    
    def setup_plot(self):
        """Configure plot appearance"""
        # Set labels with coordinate system info
        self.ax.set_xlabel('X (cm) →', fontsize=12, fontweight='bold')
        self.ax.set_ylabel('Y (cm) →', fontsize=12, fontweight='bold')
        self.ax.set_zlabel('Z (cm) ↑', fontsize=12, fontweight='bold')
        
        # Set limits (workspace)
        limit = 60
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(0, limit)
        
        # Equal aspect ratio
        self.ax.set_box_aspect([1, 1, 1])
        
        # Grid
        self.ax.grid(True, alpha=0.3)
        
        # Title
        self.ax.set_title('Prime-Arm 3D Visualization\n+X: Right | +Y: Into Screen | +Z: Up', 
                         fontsize=14, fontweight='bold')
        
        # Legend
        self.ax.legend(loc='upper right')
        
        # View angle (looking from front-right-top)
        self.ax.view_init(elev=20, azim=45)
    
    def compute_joint_positions(self, base, shoulder, elbow, wrist):
        """
        Compute 3D positions of all joints and end effector
        
        Args:
            base, shoulder, elbow, wrist: Joint angles in degrees
        
        Returns:
            List of (x, y, z) positions for each joint
        """
        # Convert to radians
        T = servo_angles_to_math_angles(base, shoulder, elbow, wrist)
        T1, T2, T3, T4 = T
        
        # Base position (origin)
        p0 = np.array([0, 0, 0])
        
        # Joint 1 position (after base rotation and vertical offset)
        p1 = np.array([0, 0, a1])
        
        # Joint 2 position (after shoulder)
        # Shoulder rotates in the plane after base rotation
        x2 = a2 * np.cos(T1) * np.cos(T2)
        y2 = a2 * np.sin(T1) * np.cos(T2)
        z2 = a1 + a2 * np.sin(T2)
        p2 = np.array([x2, y2, z2])
        
        # Joint 3 position (after elbow)
        # Elbow continues from shoulder
        angle_23 = T2 + T3 + np.pi/2  # T3 has 90° offset in DH params
        x3 = np.cos(T1) * (a2 * np.cos(T2) - a3 * np.cos(angle_23))
        y3 = np.sin(T1) * (a2 * np.cos(T2) - a3 * np.cos(angle_23))
        z3 = a1 + a2 * np.sin(T2) - a3 * np.sin(angle_23)
        p3 = np.array([x3, y3, z3])
        
        # End effector position (after wrist)
        angle_234 = T2 + T3 + T4 + np.pi  # T3 and T4 have 90° offsets
        x4 = np.cos(T1) * (a2 * np.cos(T2) - a3 * np.cos(angle_23) + a4 * np.cos(angle_234))
        y4 = np.sin(T1) * (a2 * np.cos(T2) - a3 * np.cos(angle_23) + a4 * np.cos(angle_234))
        z4 = a1 + a2 * np.sin(T2) - a3 * np.sin(angle_23) + a4 * np.sin(angle_234)
        p4 = np.array([x4, y4, z4])
        
        return [p0, p1, p2, p3, p4]
    
    def update_arm(self, base, shoulder, elbow, wrist):
        """
        Update arm visualization with new joint angles
        
        Args:
            base, shoulder, elbow, wrist: Joint angles in degrees
        """
        self.current_angles = {'b': base, 's': shoulder, 'e': elbow, 'w': wrist}
        
        # Compute joint positions
        positions = self.compute_joint_positions(base, shoulder, elbow, wrist)
        
        # Extract coordinates
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        
        # Update arm line
        self.arm_line.set_data(xs, ys)
        self.arm_line.set_3d_properties(zs)
        
        # Update end effector marker
        self.end_effector.set_data([xs[-1]], [ys[-1]])
        self.end_effector.set_3d_properties([zs[-1]])
        
        # Update joint markers (skip base)
        self.joints.set_data(xs[1:-1], ys[1:-1])
        self.joints.set_3d_properties(zs[1:-1])
        
        # Update title with current position
        end_pos = positions[-1]
        title_text = (f'Prime-Arm 3D Visualization\n'
                     f'Position: X={end_pos[0]:.1f}, Y={end_pos[1]:.1f}, Z={end_pos[2]:.1f} cm\n'
                     f'Angles: B={base}° S={shoulder}° E={elbow}° W={wrist}°')
        self.ax.set_title(title_text, fontsize=12, fontweight='bold')
        
        self.fig.canvas.draw_idle()
    
    def show(self):
        """Display the visualization"""
        plt.show()
    
    def save_frame(self, filename='robot_frame.png'):
        """Save current frame to file"""
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Saved frame to {filename}")


class RobotAnimator:
    """Animate robot movement through sequence of positions"""
    
    def __init__(self, angle_sequence):
        """
        Initialize animator
        
        Args:
            angle_sequence: List of angle dicts [{'b': 0, 's': 0, 'e': 0, 'w': 0}, ...]
        """
        self.angle_sequence = angle_sequence
        self.visualizer = RobotVisualizer()
        self.current_frame = 0
    
    def update_frame(self, frame):
        """Update animation frame"""
        angles = self.angle_sequence[frame]
        self.visualizer.update_arm(angles['b'], angles['s'], angles['e'], angles['w'])
        return self.visualizer.arm_line, self.visualizer.end_effector, self.visualizer.joints
    
    def animate(self, interval=100, repeat=True):
        """
        Start animation
        
        Args:
            interval: Milliseconds between frames
            repeat: Whether to loop animation
        """
        anim = FuncAnimation(self.visualizer.fig, self.update_frame, 
                           frames=len(self.angle_sequence),
                           interval=interval, repeat=repeat, blit=False)
        self.visualizer.show()
        return anim


def visualize_robot(base=0, shoulder=0, elbow=0, wrist=0):
    """
    Quick function to visualize robot at specific angles
    
    Args:
        base, shoulder, elbow, wrist: Joint angles in degrees
    """
    viz = RobotVisualizer()
    viz.update_arm(base, shoulder, elbow, wrist)
    viz.show()


def visualize_trajectory(start_angles, end_angles, steps=50):
    """
    Visualize smooth trajectory between two configurations
    
    Args:
        start_angles: Dict with starting angles {'b': 0, 's': 0, 'e': 0, 'w': 0}
        end_angles: Dict with ending angles
        steps: Number of interpolation steps
    """
    # Interpolate between start and end
    sequence = []
    for i in range(steps):
        t = i / (steps - 1)  # 0 to 1
        angles = {}
        for key in ['b', 's', 'e', 'w']:
            angles[key] = int(start_angles[key] + t * (end_angles[key] - start_angles[key]))
        sequence.append(angles)
    
    # Animate
    animator = RobotAnimator(sequence)
    animator.animate(interval=50, repeat=True)


if __name__ == "__main__":
    print("3D Robot Arm Visualizer")
    print("=" * 50)
    print("\nCoordinate System:")
    print("  +X: Right")
    print("  +Y: Into screen")
    print("  +Z: Up")
    print("\n" + "=" * 50)
    
    # Example 1: Static visualization
    print("\nExample 1: Static pose at 90° angles")
    print("Close the window to continue...")
    visualize_robot(base=90, shoulder=90, elbow=90, wrist=90)
    
    # Example 2: Trajectory animation
    print("\nExample 2: Animated trajectory")
    print("Animating from home to extended position...")
    start = {'b': 0, 's': 0, 'e': 0, 'w': 0}
    end = {'b': 90, 's': 120, 'e': 60, 'w': 90}
    visualize_trajectory(start, end, steps=50)
