#!/usr/bin/env python3
"""
GUI Controller for Prime-Arm Robot
Simple interface with sliders to control each servo
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from raspberry_pi_controller import RobotArmController


class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Prime-Arm Controller")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        # Robot controller
        self.robot = None
        self.connected = False
        self.update_thread = None
        self.running = False
        
        # Servo configurations
        self.servos = {
            'b': {'name': 'Base', 'min': 0, 'max': 180, 'current': 0},
            's': {'name': 'Shoulder', 'min': 0, 'max': 180, 'current': 0},
            'e': {'name': 'Elbow', 'min': 0, 'max': 180, 'current': 0},
            'w': {'name': 'Wrist', 'min': 0, 'max': 180, 'current': 0},
            't': {'name': 'Twist', 'min': 0, 'max': 180, 'current': 0},
            'g': {'name': 'Gripper', 'min': 0, 'max': 80, 'current': 80}
        }
        
        # Target angles (from sliders)
        self.target_angles = {k: v['current'] for k, v in self.servos.items()}
        
        # Track if we're currently sending commands
        self.sending_command = False
        
        self.create_widgets()
        
    def create_widgets(self):
        """Create GUI widgets"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        
        # Title
        title = ttk.Label(main_frame, text="Prime-Arm Robot Controller", 
                         font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, pady=(0, 10))
        
        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        conn_frame.columnconfigure(1, weight=1)
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.port_entry = ttk.Entry(conn_frame, width=20)
        self.port_entry.insert(0, "COM4" if tk.sys.platform.startswith('win') else "/dev/ttyACM0")
        self.port_entry.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(0, 10))
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2)
        
        self.status_label = ttk.Label(conn_frame, text="● Disconnected", foreground="red")
        self.status_label.grid(row=0, column=3, padx=(10, 0))
        
        # Servo controls frame
        servo_frame = ttk.LabelFrame(main_frame, text="Servo Controls", padding="10")
        servo_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        servo_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)
        
        self.sliders = {}
        self.angle_labels = {}
        
        # Create slider for each servo
        row = 0
        for servo_id, config in self.servos.items():
            # Servo name label
            name_label = ttk.Label(servo_frame, text=f"{config['name']}:", 
                                  font=('Arial', 10, 'bold'), width=10)
            name_label.grid(row=row, column=0, sticky=tk.W, pady=5)
            
            # Angle display (create BEFORE slider to avoid callback issues)
            angle_label = ttk.Label(servo_frame, text=f"{config['current']}°", 
                                   font=('Arial', 10), width=6)
            angle_label.grid(row=row, column=2, sticky=tk.E)
            self.angle_labels[servo_id] = angle_label
            
            # Slider (created after angle_label so callback doesn't fail)
            slider = ttk.Scale(servo_frame, from_=config['min'], to=config['max'],
                             orient=tk.HORIZONTAL, command=lambda v, s=servo_id: self.on_slider_change(s, v))
            slider.set(config['current'])
            slider.grid(row=row, column=1, sticky=(tk.W, tk.E), padx=(10, 10))
            self.sliders[servo_id] = slider
            
            # Min/Max labels
            range_label = ttk.Label(servo_frame, text=f"({config['min']}° - {config['max']}°)", 
                                   font=('Arial', 8), foreground="gray")
            range_label.grid(row=row, column=3, sticky=tk.W, padx=(5, 0))
            
            row += 1
        
        # Control buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, pady=(0, 10))
        
        self.home_btn = ttk.Button(button_frame, text="Home Position", 
                                   command=self.go_home, state='disabled')
        self.home_btn.grid(row=0, column=0, padx=5)
        
        self.refresh_btn = ttk.Button(button_frame, text="Refresh Angles", 
                                     command=self.refresh_angles, state='disabled')
        self.refresh_btn.grid(row=0, column=1, padx=5)
        
        # Smooth motion checkbox
        self.smooth_var = tk.BooleanVar(value=True)
        smooth_check = ttk.Checkbutton(button_frame, text="Smooth Motion (1° steps)", 
                                      variable=self.smooth_var)
        smooth_check.grid(row=0, column=2, padx=20)
        
        # Status bar
        status_bar = ttk.Frame(main_frame, relief=tk.SUNKEN)
        status_bar.grid(row=4, column=0, sticky=(tk.W, tk.E))
        
        self.info_label = ttk.Label(status_bar, text="Ready. Please connect to Arduino.", 
                                   font=('Arial', 9))
        self.info_label.pack(side=tk.LEFT, padx=5, pady=2)
        
    def on_slider_change(self, servo_id, value):
        """Handle slider value change"""
        angle = int(float(value))
        self.angle_labels[servo_id].config(text=f"{angle}°")
        self.target_angles[servo_id] = angle
        
        # Only send command if connected and not currently sending
        if self.connected and not self.sending_command:
            # Schedule command with small delay to avoid flooding
            self.root.after(50, lambda: self.send_servo_command(servo_id, angle))
    
    def send_servo_command(self, servo_id, angle):
        """Send command to move servo"""
        if not self.connected or self.sending_command:
            return
        
        self.sending_command = True
        
        def send():
            try:
                smooth = self.smooth_var.get()
                self.robot.send_command({servo_id: angle}, smooth=smooth, display_progress=False)
                self.update_info(f"Moved {self.servos[servo_id]['name']} to {angle}°")
            except Exception as e:
                self.update_info(f"Error: {e}")
            finally:
                self.sending_command = False
        
        # Send in background thread
        threading.Thread(target=send, daemon=True).start()
    
    def toggle_connection(self):
        """Connect or disconnect from Arduino"""
        if not self.connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        """Connect to Arduino"""
        port = self.port_entry.get()
        
        try:
            self.update_info("Connecting...")
            self.robot = RobotArmController(port=port, baudrate=115200, step_delay=0.015)
            
            if self.robot.connect():
                self.connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="● Connected", foreground="green")
                self.home_btn.config(state='normal')
                self.refresh_btn.config(state='normal')
                self.port_entry.config(state='disabled')
                
                # Get initial angles and update sliders
                self.refresh_angles()
                self.update_info("Connected successfully!")
                
                # Start update thread
                self.running = True
                self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
                self.update_thread.start()
            else:
                messagebox.showerror("Connection Error", "Failed to connect to Arduino")
                self.update_info("Connection failed")
                
        except Exception as e:
            messagebox.showerror("Error", f"Connection error: {e}")
            self.update_info(f"Error: {e}")
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.robot:
            self.running = False
            self.robot.disconnect()
            self.robot = None
        
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="● Disconnected", foreground="red")
        self.home_btn.config(state='disabled')
        self.refresh_btn.config(state='disabled')
        self.port_entry.config(state='normal')
        self.update_info("Disconnected")
    
    def go_home(self):
        """Move all servos to home position"""
        if not self.connected:
            return
        
        def move_home():
            try:
                self.update_info("Moving to home position...")
                smooth = self.smooth_var.get()
                self.robot.home_position(smooth=smooth, display_progress=False)
                
                # Update sliders to home position
                home_angles = {'b': 0, 's': 0, 'e': 0, 'w': 0, 't': 0, 'g': 80}
                for servo_id, angle in home_angles.items():
                    self.root.after(0, lambda s=servo_id, a=angle: self.sliders[s].set(a))
                
                self.update_info("Home position reached")
            except Exception as e:
                self.update_info(f"Error: {e}")
        
        threading.Thread(target=move_home, daemon=True).start()
    
    def refresh_angles(self):
        """Get current angles from Arduino and update sliders"""
        if not self.connected:
            return
        
        try:
            current_angles = self.robot.get_current_angles()
            
            # Update sliders without triggering commands
            for servo_id, angle in current_angles.items():
                if servo_id in self.sliders:
                    self.sliders[servo_id].set(angle)
                    self.angle_labels[servo_id].config(text=f"{angle}°")
            
            self.update_info("Angles refreshed")
        except Exception as e:
            self.update_info(f"Error: {e}")
    
    def update_loop(self):
        """Background thread to periodically update angles"""
        while self.running and self.connected:
            time.sleep(2)  # Update every 2 seconds
            if self.connected and not self.sending_command:
                try:
                    current_angles = self.robot.get_current_angles()
                    # Update labels only (not sliders to avoid interrupting user)
                    for servo_id, angle in current_angles.items():
                        if servo_id in self.angle_labels:
                            self.root.after(0, lambda s=servo_id, a=angle: 
                                          self.angle_labels[s].config(text=f"{a}°"))
                except:
                    pass
    
    def update_info(self, message):
        """Update info label"""
        self.root.after(0, lambda: self.info_label.config(text=message))
    
    def on_closing(self):
        """Handle window closing"""
        if self.connected:
            self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = RobotArmGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
