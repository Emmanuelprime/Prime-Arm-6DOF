#!/usr/bin/env python3
"""
Inverse Kinematics Testing GUI
Generate random positions and test IK solver
"""

import tkinter as tk
from tkinter import ttk
import random
from inverse_kinematics_servo import calculate_ik, forward_kinematics, servo_angles_to_math_angles

# Robot workspace limits (cm)
X_MIN, X_MAX = -40, 40
Y_MIN, Y_MAX = -40, 40
Z_MIN, Z_MAX = 20, 60


class IKTesterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("IK Tester - Prime-Arm")
        self.root.geometry("600x500")
        
        self.create_widgets()
    
    def create_widgets(self):
        # Title
        title_label = tk.Label(self.root, text="Inverse Kinematics Tester", 
                               font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        # Target Position Frame
        target_frame = ttk.LabelFrame(self.root, text="Target Position (cm)", padding=10)
        target_frame.pack(padx=20, pady=10, fill="x")
        
        # X input
        x_frame = tk.Frame(target_frame)
        x_frame.pack(fill="x", pady=5)
        tk.Label(x_frame, text="X:", width=8, anchor="w").pack(side="left")
        self.x_var = tk.StringVar(value="0.0")
        self.x_entry = tk.Entry(x_frame, textvariable=self.x_var, width=15)
        self.x_entry.pack(side="left", padx=5)
        tk.Label(x_frame, text=f"({X_MIN} to {X_MAX})").pack(side="left")
        
        # Y input
        y_frame = tk.Frame(target_frame)
        y_frame.pack(fill="x", pady=5)
        tk.Label(y_frame, text="Y:", width=8, anchor="w").pack(side="left")
        self.y_var = tk.StringVar(value="0.0")
        self.y_entry = tk.Entry(y_frame, textvariable=self.y_var, width=15)
        self.y_entry.pack(side="left", padx=5)
        tk.Label(y_frame, text=f"({Y_MIN} to {Y_MAX})").pack(side="left")
        
        # Z input
        z_frame = tk.Frame(target_frame)
        z_frame.pack(fill="x", pady=5)
        tk.Label(z_frame, text="Z:", width=8, anchor="w").pack(side="left")
        self.z_var = tk.StringVar(value="40.0")
        self.z_entry = tk.Entry(z_frame, textvariable=self.z_var, width=15)
        self.z_entry.pack(side="left", padx=5)
        tk.Label(z_frame, text=f"({Z_MIN} to {Z_MAX})").pack(side="left")
        
        # Buttons Frame
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)
        
        self.random_btn = tk.Button(button_frame, text="Generate Random Position",
                                    command=self.generate_random, 
                                    bg="#4CAF50", fg="white", 
                                    padx=15, pady=8, font=("Arial", 10, "bold"))
        self.random_btn.pack(side="left", padx=5)
        
        self.solve_btn = tk.Button(button_frame, text="Solve IK",
                                   command=self.solve_ik,
                                   bg="#2196F3", fg="white",
                                   padx=15, pady=8, font=("Arial", 10, "bold"))
        self.solve_btn.pack(side="left", padx=5)
        
        # Results Frame
        results_frame = ttk.LabelFrame(self.root, text="Results", padding=10)
        results_frame.pack(padx=20, pady=10, fill="both", expand=True)
        
        # Results text area
        self.results_text = tk.Text(results_frame, height=15, width=60, 
                                    font=("Courier", 10), bg="#f5f5f5")
        self.results_text.pack(fill="both", expand=True)
        
        # Initial message
        self.results_text.insert("1.0", "Ready to test IK solver.\n\n"
                                       "1. Click 'Generate Random Position' for a random target\n"
                                       "2. Or enter your own X, Y, Z values\n"
                                       "3. Click 'Solve IK' to compute servo angles\n\n"
                                       f"Workspace limits:\n"
                                       f"  X: {X_MIN} to {X_MAX} cm\n"
                                       f"  Y: {Y_MIN} to {Y_MAX} cm\n"
                                       f"  Z: {Z_MIN} to {Z_MAX} cm")
        self.results_text.config(state="disabled")
    
    def generate_random(self):
        """Generate random target position within workspace"""
        x = round(random.uniform(X_MIN, X_MAX), 2)
        y = round(random.uniform(Y_MIN, Y_MAX), 2)
        z = round(random.uniform(Z_MIN, Z_MAX), 2)
        
        self.x_var.set(str(x))
        self.y_var.set(str(y))
        self.z_var.set(str(z))
        
        self.update_results(f"Generated random position: ({x:.2f}, {y:.2f}, {z:.2f}) cm\n"
                          f"Click 'Solve IK' to compute servo angles.")
    
    def solve_ik(self):
        """Solve IK for current target position"""
        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            z = float(self.z_var.get())
        except ValueError:
            self.update_results("ERROR: Invalid input. Please enter numeric values.")
            return
        
        # Check workspace limits
        if not (X_MIN <= x <= X_MAX):
            self.update_results(f"WARNING: X={x:.2f} is outside limits ({X_MIN} to {X_MAX})")
        if not (Y_MIN <= y <= Y_MAX):
            self.update_results(f"WARNING: Y={y:.2f} is outside limits ({Y_MIN} to {Y_MAX})")
        if not (Z_MIN <= z <= Z_MAX):
            self.update_results(f"WARNING: Z={z:.2f} is outside limits ({Z_MIN} to {Z_MAX})")
        
        # Solve IK
        self.update_results(f"\n{'='*50}\nSOLVING IK\n{'='*50}")
        self.update_results(f"Target Position: ({x:.2f}, {y:.2f}, {z:.2f}) cm")
        
        servo_angles, success, error = calculate_ik(x, y, z)
        
        # Display servo angles
        result_text = f"\nServo Angles (0-180°):\n"
        result_text += f"  Base (b):      {servo_angles['b']:3d}°\n"
        result_text += f"  Shoulder (s):  {servo_angles['s']:3d}°\n"
        result_text += f"  Elbow (e):     {servo_angles['e']:3d}°\n"
        result_text += f"  Wrist (w):     {servo_angles['w']:3d}°\n"
        
        # Forward kinematics verification
        T = servo_angles_to_math_angles(
            servo_angles['b'], servo_angles['s'],
            servo_angles['e'], servo_angles['w']
        )
        xf, yf, zf = forward_kinematics(*T)
        
        result_text += f"\nForward Kinematics Verification:\n"
        result_text += f"  Target:   ({x:7.2f}, {y:7.2f}, {z:7.2f}) cm\n"
        result_text += f"  Achieved: ({xf:7.2f}, {yf:7.2f}, {zf:7.2f}) cm\n"
        result_text += f"  Error:    {error:7.4f} cm\n"
        
        # Status
        result_text += f"\nStatus: "
        if success:
            result_text += "✓ SUCCESS (error < 0.5 cm)\n"
        else:
            result_text += "✗ FAILED (error >= 0.5 cm)\n"
        
        # Serial command format
        result_text += f"\nSerial Command:\n"
        result_text += f"  b{servo_angles['b']},s{servo_angles['s']},"
        result_text += f"e{servo_angles['e']},w{servo_angles['w']}\n"
        
        self.update_results(result_text)
    
    def update_results(self, text):
        """Update results text area"""
        self.results_text.config(state="normal")
        self.results_text.delete("1.0", "end")
        self.results_text.insert("1.0", text)
        self.results_text.config(state="disabled")


def main():
    root = tk.Tk()
    app = IKTesterGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
