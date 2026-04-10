#!/usr/bin/env python3
"""
Servo Test GUI for Prime-Arm Robot
Test and calibrate all servo angles individually
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
from raspberry_pi_controller import RobotArmController


class ServoTestGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Prime-Arm Servo Tester")
        self.root.geometry("700x650")
        
        self.robot = None
        self.servo_vars = {}
        self.servo_labels = {}
        self.auto_update = tk.BooleanVar(value=True)
        self._pending_rt = {}   # pending real-time servo values
        self._rt_timer = None   # debounce timer id
        
        self.create_widgets()
        self.connect_robot()
    
    def create_widgets(self):
        """Create GUI widgets"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title = ttk.Label(main_frame, text="🎛️ Prime-Arm Servo Tester", 
                         font=('Arial', 18, 'bold'))
        title.pack(pady=(0, 10))
        
        # Connection Status
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=(0, 15))
        
        self.connection_label = ttk.Label(status_frame, text="⚫ Disconnected", 
                                         font=('Arial', 10, 'bold'), foreground='red')
        self.connection_label.pack(side=tk.LEFT)
        
        ttk.Button(status_frame, text="Reconnect", 
                  command=self.connect_robot).pack(side=tk.LEFT, padx=10)
        
        self.pos_label = ttk.Label(status_frame, text="Position: --", 
                                  font=('Arial', 9), foreground='gray')
        self.pos_label.pack(side=tk.RIGHT, padx=10)
        
        # Servo Controls Frame
        servos_frame = ttk.LabelFrame(main_frame, text="Servo Controls", padding="10")
        servos_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # Define servo parameters: (name, key, min, max, default, color)
        servos = [
            ("Base", "b", 0, 180, 90, "#FF6B6B"),
            ("Shoulder", "s", 0, 180, 90, "#4ECDC4"),
            ("Elbow", "e", 0, 180, 90, "#45B7D1"),
            ("Wrist", "w", 0, 180, 90, "#96CEB4"),
            ("Twist", "t", 0, 180, 0, "#FFEAA7"),
            ("Gripper", "g", 30, 80, 30, "#DFE6E9"),
        ]
        
        for i, (name, key, min_val, max_val, default, color) in enumerate(servos):
            self.create_servo_control(servos_frame, name, key, min_val, max_val, default, color, i)
        
        # Control Buttons
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        left_controls = ttk.Frame(control_frame)
        left_controls.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        ttk.Button(left_controls, text="📤 Send All", 
                  command=self.send_all_servos, 
                  style='Accent.TButton').pack(side=tk.LEFT, padx=5)
        
        ttk.Button(left_controls, text="🏠 Home Position", 
                  command=self.go_home).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(left_controls, text="📍 Read Current", 
                  command=self.read_current_angles).pack(side=tk.LEFT, padx=5)
        
        ttk.Checkbutton(left_controls, text="Real-time",
                       variable=self.auto_update,
                       command=self.toggle_auto_update).pack(side=tk.LEFT, padx=10)
        
        # Preset Positions
        presets_frame = ttk.LabelFrame(main_frame, text="Preset Positions", padding="10")
        presets_frame.pack(fill=tk.X, pady=(0, 10))
        
        presets_grid = ttk.Frame(presets_frame)
        presets_grid.pack()
        
        presets = [
            ("Home", {'b': 90, 's': 130, 'e': 180, 'w': 180, 't': 180, 'g': 80}),
            ("Neutral", {'b': 90, 's': 90, 'e': 90, 'w': 90, 't': 0, 'g': 30}),
            ("Reach Forward", {'b': 90, 's': 120, 'e': 60, 'w': 90, 't': 0, 'g': 30}),
            ("Grip Test", {'b': 90, 's': 90, 'e': 90, 'w': 90, 't': 0, 'g': 80}),
        ]
        
        for i, (name, angles) in enumerate(presets):
            row = i // 2
            col = i % 2
            ttk.Button(presets_grid, text=name, 
                      command=lambda a=angles: self.load_preset(a),
                      width=20).grid(row=row, column=col, padx=5, pady=5)
        
        # Test Sequences
        test_frame = ttk.LabelFrame(main_frame, text="Test Sequences", padding="10")
        test_frame.pack(fill=tk.X, pady=(0, 10))
        
        test_buttons = ttk.Frame(test_frame)
        test_buttons.pack()
        
        ttk.Button(test_buttons, text="🔄 Sweep All Servos", 
                  command=self.sweep_all).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(test_buttons, text="🎯 Test Base Rotation", 
                  command=self.test_base).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(test_buttons, text="💪 Test Arm Movement", 
                  command=self.test_arm).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(test_buttons, text="✋ Test Gripper", 
                  command=self.test_gripper).grid(row=0, column=3, padx=5, pady=5)
        
        # Status Bar
        status_bar = ttk.Frame(main_frame, relief=tk.SUNKEN)
        status_bar.pack(fill=tk.X)
        
        self.status_label = ttk.Label(status_bar, text="Ready", 
                                     font=('Arial', 9), foreground='green')
        self.status_label.pack(side=tk.LEFT, padx=5, pady=3)
    
    def create_servo_control(self, parent, name, key, min_val, max_val, default, color, row):
        """Create a servo control slider"""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=5)
        
        # Servo name with colored indicator
        name_frame = ttk.Frame(frame)
        name_frame.pack(side=tk.LEFT, padx=5)
        
        color_label = tk.Label(name_frame, text="●", font=('Arial', 12), fg=color)
        color_label.pack(side=tk.LEFT)
        
        label = ttk.Label(name_frame, text=f"{name}:", font=('Arial', 10, 'bold'), width=10)
        label.pack(side=tk.LEFT, padx=5)
        
        # Slider
        var = tk.IntVar(value=default)
        self.servo_vars[key] = var
        
        slider = ttk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL, 
                          variable=var, length=300,
                          command=lambda v, k=key: self.on_slider_change(k, v))
        slider.pack(side=tk.LEFT, padx=5)
        
        # Value display
        value_label = ttk.Label(frame, text=f"{default}°", font=('Arial', 10), width=6)
        value_label.pack(side=tk.LEFT, padx=5)
        self.servo_labels[key] = value_label
        
        # Range display
        range_label = ttk.Label(frame, text=f"[{min_val}-{max_val}°]", 
                               font=('Arial', 8), foreground='gray')
        range_label.pack(side=tk.LEFT, padx=5)
        
        # Direct send button
        ttk.Button(frame, text="Send", width=6,
                  command=lambda k=key: self.send_single_servo(k)).pack(side=tk.LEFT, padx=5)
        
        # Update value label when slider changes
        var.trace('w', lambda *args, k=key: self.update_value_label(k))
    
    def update_value_label(self, key):
        """Update the value label for a servo"""
        value = self.servo_vars[key].get()
        self.servo_labels[key].config(text=f"{value}°")
    
    def on_slider_change(self, key, value):
        """Handle slider change - debounced real-time send"""
        if self.auto_update.get() and self.robot:
            self._pending_rt[key] = int(float(value))
            # Cancel existing timer and reschedule (50 ms debounce)
            if self._rt_timer is not None:
                self.root.after_cancel(self._rt_timer)
            self._rt_timer = self.root.after(50, self._flush_rt_commands)

    def _flush_rt_commands(self):
        """Send all pending real-time servo commands in one serial packet"""
        self._rt_timer = None
        if not self._pending_rt or not self.robot:
            return
        to_send = dict(self._pending_rt)
        self._pending_rt.clear()

        def send():
            try:
                self.robot._send_raw_command(to_send)
                keys = ', '.join(f"{k.upper()}={v}°" for k, v in to_send.items())
                self.status_label.config(text=f"RT → {keys}", foreground='blue')
            except Exception as e:
                self.status_label.config(text=f"RT error: {e}", foreground='red')

        threading.Thread(target=send, daemon=True).start()

    def toggle_auto_update(self):
        """Toggle auto update mode"""
        if self.auto_update.get():
            self.status_label.config(text="Real-time mode ON — servos follow sliders instantly",
                                   foreground='blue')
        else:
            self.status_label.config(text="Real-time mode OFF — use Send buttons",
                                   foreground='gray')
    
    def connect_robot(self):
        """Connect to robot"""
        if self.robot:
            self.robot.disconnect()
        
        try:
            from raspberry_pi_controller import DEFAULT_PORT
            self.robot = RobotArmController(port=DEFAULT_PORT)
            if self.robot.connect():
                self.connection_label.config(text="🟢 Connected", foreground='green')
                self.status_label.config(text="Robot connected — Real-time mode ON", foreground='green')
                self.read_current_angles()
            else:
                self.connection_label.config(text="🔴 Connection Failed", foreground='red')
                self.status_label.config(text="Failed to connect to robot", foreground='red')
                self.robot = None
        except Exception as e:
            self.connection_label.config(text="🔴 Error", foreground='red')
            self.status_label.config(text=f"Error: {e}", foreground='red')
            self.robot = None
    
    def send_single_servo(self, key):
        """Send command for a single servo"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        angle = self.servo_vars[key].get()
        
        def send():
            try:
                self.robot.send_command({key: angle}, smooth=False, display_progress=False)
                self.status_label.config(text=f"{key.upper()} moved to {angle}°", foreground='green')
                self.update_position_display()
            except Exception as e:
                self.status_label.config(text=f"Error: {e}", foreground='red')
        
        threading.Thread(target=send, daemon=True).start()
    
    def send_all_servos(self):
        """Send command for all servos"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        command = {key: var.get() for key, var in self.servo_vars.items()}
        
        def send():
            try:
                self.status_label.config(text="Sending all servo commands...", foreground='blue')
                self.robot.send_command(command, smooth=True, display_progress=False)
                self.status_label.config(text="All servos moved successfully", foreground='green')
                self.update_position_display()
            except Exception as e:
                self.status_label.config(text=f"Error: {e}", foreground='red')
        
        threading.Thread(target=send, daemon=True).start()
    
    def read_current_angles(self):
        """Read current angles from robot and update sliders"""
        if not self.robot:
            return
        
        try:
            angles = self.robot.get_current_angles()
            for key, var in self.servo_vars.items():
                if key in angles:
                    var.set(angles[key])
            self.status_label.config(text="Current angles loaded from robot", foreground='green')
            self.update_position_display()
        except Exception as e:
            self.status_label.config(text=f"Error reading angles: {e}", foreground='red')
    
    def update_position_display(self):
        """Update end effector position display"""
        if not self.robot:
            return
        
        try:
            x, y, z = self.robot.get_end_effector_position()
            self.pos_label.config(text=f"Position: ({x:.1f}, {y:.1f}, {z:.1f}) cm")
        except Exception:
            pass
    
    def go_home(self):
        """Move to home position"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        def move():
            try:
                self.status_label.config(text="Moving to home position...", foreground='blue')
                self.robot.home_position(smooth=True, display_progress=False)
                self.read_current_angles()
                self.status_label.config(text="Home position reached", foreground='green')
            except Exception as e:
                self.status_label.config(text=f"Error: {e}", foreground='red')
        
        threading.Thread(target=move, daemon=True).start()
    
    def load_preset(self, angles):
        """Load a preset position"""
        for key, value in angles.items():
            if key in self.servo_vars:
                self.servo_vars[key].set(value)
        self.status_label.config(text="Preset loaded - click 'Send All' to execute", foreground='blue')
    
    def sweep_all(self):
        """Sweep all servos through their range"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        def sweep():
            try:
                import time
                self.status_label.config(text="Sweeping all servos...", foreground='blue')
                
                # Define safe sweep ranges
                sweeps = [
                    ('b', 0, 180, 30),
                    ('s', 45, 135, 30),
                    ('e', 45, 135, 30),
                    ('w', 45, 135, 30),
                    ('t', 0, 180, 30),
                    ('g', 0, 80, 20),
                ]
                
                for key, min_angle, max_angle, step in sweeps:
                    self.status_label.config(text=f"Sweeping {key.upper()}...", foreground='blue')
                    
                    # Sweep forward
                    for angle in range(min_angle, max_angle + 1, step):
                        if key in self.servo_vars:
                            self.servo_vars[key].set(angle)
                        self.robot.send_command({key: angle}, smooth=False, display_progress=False)
                        time.sleep(0.3)
                    
                    # Sweep back
                    for angle in range(max_angle, min_angle - 1, -step):
                        if key in self.servo_vars:
                            self.servo_vars[key].set(angle)
                        self.robot.send_command({key: angle}, smooth=False, display_progress=False)
                        time.sleep(0.3)
                
                self.status_label.config(text="Sweep complete!", foreground='green')
                self.read_current_angles()
                
            except Exception as e:
                self.status_label.config(text=f"Error during sweep: {e}", foreground='red')
        
        threading.Thread(target=sweep, daemon=True).start()
    
    def test_base(self):
        """Test base rotation"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        def test():
            try:
                import time
                self.status_label.config(text="Testing base rotation...", foreground='blue')
                
                positions = [0, 45, 90, 135, 180, 90, 0]
                for angle in positions:
                    self.servo_vars['b'].set(angle)
                    self.robot.send_command({'b': angle}, smooth=True, display_progress=False)
                    time.sleep(1)
                
                self.status_label.config(text="Base test complete", foreground='green')
            except Exception as e:
                self.status_label.config(text=f"Error: {e}", foreground='red')
        
        threading.Thread(target=test, daemon=True).start()
    
    def test_arm(self):
        """Test arm movement"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        def test():
            try:
                import time
                self.status_label.config(text="Testing arm movement...", foreground='blue')
                
                # Sequence of arm positions
                positions = [
                    {'s': 90, 'e': 90, 'w': 90},
                    {'s': 120, 'e': 60, 'w': 90},
                    {'s': 60, 'e': 120, 'w': 90},
                    {'s': 90, 'e': 90, 'w': 120},
                    {'s': 90, 'e': 90, 'w': 60},
                    {'s': 90, 'e': 90, 'w': 90},
                ]
                
                for pos in positions:
                    for key, angle in pos.items():
                        self.servo_vars[key].set(angle)
                    self.robot.send_command(pos, smooth=True, display_progress=False)
                    time.sleep(1.5)
                
                self.status_label.config(text="Arm test complete", foreground='green')
            except Exception as e:
                self.status_label.config(text=f"Error: {e}", foreground='red')
        
        threading.Thread(target=test, daemon=True).start()
    
    def test_gripper(self):
        """Test gripper open/close"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        def test():
            try:
                import time
                self.status_label.config(text="Testing gripper...", foreground='blue')
                
                for i in range(3):
                    # Close
                    self.servo_vars['g'].set(80)
                    self.robot.send_command({'g': 80}, smooth=True, display_progress=False)
                    time.sleep(0.8)
                    
                    # Open
                    self.servo_vars['g'].set(30)
                    self.robot.send_command({'g': 30}, smooth=True, display_progress=False)
                    time.sleep(0.8)
                
                self.status_label.config(text="Gripper test complete", foreground='green')
            except Exception as e:
                self.status_label.config(text=f"Error: {e}", foreground='red')
        
        threading.Thread(target=test, daemon=True).start()


def main():
    root = tk.Tk()
    app = ServoTestGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
