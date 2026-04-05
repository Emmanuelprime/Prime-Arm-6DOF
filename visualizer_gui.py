#!/usr/bin/env python3
"""
Integrated 3D Visualizer for GUI Controller
Opens in separate window and updates in real-time
"""

import tkinter as tk
from tkinter import ttk
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from inverse_kinematics_servo import servo_angles_to_math_angles

# Link lengths
a1 = 16.4
a2 = 10.5
a3 = 14.8
a4 = 18.0


class RobotVisualizerWindow:
    """Standalone 3D visualization window for robot arm"""
    
    def __init__(self, parent=None):
        """Initialize visualization window"""
        self.window = tk.Toplevel(parent) if parent else tk.Tk()
        self.window.title("Prime-Arm 3D Visualization")
        self.window.geometry("900x700")
        
        # Current angles
        self.current_angles = {'b': 0, 's': 0, 'e': 0, 'w': 0}
        
        # Set is_open before create_widgets (needed by update_arm)
        self.is_open = True
        
        self.create_widgets()
        
        # Protocol for window close
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_widgets(self):
        """Create GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.window, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Info frame
        info_frame = ttk.Frame(main_frame)
        info_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(info_frame, text="Coordinate System: ", 
                 font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
        ttk.Label(info_frame, text="+X: Right  |  +Y: Into Screen  |  +Z: Up", 
                 font=('Arial', 10)).pack(side=tk.LEFT, padx=10)
        
        # Position info
        self.position_label = ttk.Label(info_frame, text="Position: X=0.0, Y=0.0, Z=0.0 cm",
                                       font=('Arial', 10), foreground='blue')
        self.position_label.pack(side=tk.RIGHT)
        
        # Matplotlib figure
        self.fig = Figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=main_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Initialize plot
        self.arm_line, = self.ax.plot([], [], [], 'o-', linewidth=4, 
                                      markersize=8, color='#2196F3', label='Robot Arm')
        self.end_effector, = self.ax.plot([], [], [], 'o', markersize=14, 
                                          color='#F44336', label='End Effector')
        self.joints, = self.ax.plot([], [], [], 'o', markersize=10, 
                                    color='#FF9800', label='Joints')
        
        self.setup_plot()
        
        # Control frame
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Button(control_frame, text="Reset View", 
                  command=self.reset_view).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(control_frame, text="Top View", 
                  command=lambda: self.set_view(90, -90)).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(control_frame, text="Side View", 
                  command=lambda: self.set_view(0, 0)).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(control_frame, text="Front View", 
                  command=lambda: self.set_view(0, -90)).pack(side=tk.LEFT, padx=5)
        
        # Update with initial position
        self.update_arm(0, 0, 0, 0)
    
    def setup_plot(self):
        """Configure plot appearance"""
        self.ax.set_xlabel('X (cm) →', fontsize=11, fontweight='bold')
        self.ax.set_ylabel('Y (cm) →', fontsize=11, fontweight='bold')
        self.ax.set_zlabel('Z (cm) ↑', fontsize=11, fontweight='bold')
        
        limit = 60
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(0, limit)
        
        self.ax.set_box_aspect([1, 1, 1])
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right', fontsize=9)
        
        # Default view
        self.ax.view_init(elev=20, azim=45)
    
    def compute_joint_positions(self, base, shoulder, elbow, wrist):
        """Compute 3D positions of all joints"""
        T = servo_angles_to_math_angles(base, shoulder, elbow, wrist)
        T1, T2, T3, T4 = T
        
        p0 = np.array([0, 0, 0])
        p1 = np.array([0, 0, a1])
        
        x2 = a2 * np.cos(T1) * np.cos(T2)
        y2 = a2 * np.sin(T1) * np.cos(T2)
        z2 = a1 + a2 * np.sin(T2)
        p2 = np.array([x2, y2, z2])
        
        angle_23 = T2 + T3 + np.pi/2
        x3 = np.cos(T1) * (a2 * np.cos(T2) - a3 * np.cos(angle_23))
        y3 = np.sin(T1) * (a2 * np.cos(T2) - a3 * np.cos(angle_23))
        z3 = a1 + a2 * np.sin(T2) - a3 * np.sin(angle_23)
        p3 = np.array([x3, y3, z3])
        
        angle_234 = T2 + T3 + T4 + np.pi
        x4 = np.cos(T1) * (a2 * np.cos(T2) - a3 * np.cos(angle_23) + a4 * np.cos(angle_234))
        y4 = np.sin(T1) * (a2 * np.cos(T2) - a3 * np.cos(angle_23) + a4 * np.cos(angle_234))
        z4 = a1 + a2 * np.sin(T2) - a3 * np.sin(angle_23) + a4 * np.sin(angle_234)
        p4 = np.array([x4, y4, z4])
        
        return [p0, p1, p2, p3, p4]
    
    def update_arm(self, base, shoulder, elbow, wrist):
        """Update arm visualization"""
        if not self.is_open:
            return
        
        self.current_angles = {'b': base, 's': shoulder, 'e': elbow, 'w': wrist}
        
        positions = self.compute_joint_positions(base, shoulder, elbow, wrist)
        
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        
        self.arm_line.set_data(xs, ys)
        self.arm_line.set_3d_properties(zs)
        
        self.end_effector.set_data([xs[-1]], [ys[-1]])
        self.end_effector.set_3d_properties([zs[-1]])
        
        self.joints.set_data(xs[1:-1], ys[1:-1])
        self.joints.set_3d_properties(zs[1:-1])
        
        # Update position label
        end_pos = positions[-1]
        self.position_label.config(
            text=f"Position: X={end_pos[0]:.1f}, Y={end_pos[1]:.1f}, Z={end_pos[2]:.1f} cm | "
                 f"Angles: B={base}° S={shoulder}° E={elbow}° W={wrist}°")
        
        self.canvas.draw_idle()
    
    def reset_view(self):
        """Reset to default view angle"""
        self.ax.view_init(elev=20, azim=45)
        self.canvas.draw()
    
    def set_view(self, elev, azim):
        """Set specific view angle"""
        self.ax.view_init(elev=elev, azim=azim)
        self.canvas.draw()
    
    def on_closing(self):
        """Handle window close"""
        self.is_open = False
        self.window.destroy()
    
    def show(self):
        """Show window"""
        self.window.mainloop()


def test_visualizer():
    """Test the visualizer standalone"""
    viz = RobotVisualizerWindow()
    
    # Test animation
    import time
    angles_sequence = [
        {'b': 0, 's': 0, 'e': 0, 'w': 0},
        {'b': 45, 's': 30, 'e': 30, 'w': 30},
        {'b': 90, 's': 60, 'e': 60, 'w': 60},
        {'b': 135, 's': 90, 'e': 90, 'w': 90},
        {'b': 90, 's': 120, 'e': 60, 'w': 90},
    ]
    
    def animate():
        for angles in angles_sequence:
            if not viz.is_open:
                break
            viz.update_arm(angles['b'], angles['s'], angles['e'], angles['w'])
            time.sleep(1)
    
    # Start animation in background
    threading.Thread(target=animate, daemon=True).start()
    
    viz.show()


if __name__ == "__main__":
    test_visualizer()
