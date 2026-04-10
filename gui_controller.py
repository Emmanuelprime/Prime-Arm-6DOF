#!/usr/bin/env python3
"""
GUI Controller for Prime-Arm Robot
Simple interface with sliders to control each servo
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import random
from raspberry_pi_controller import RobotArmController
try:
    from visualizer_gui import RobotVisualizerWindow
    VISUALIZER_AVAILABLE = True
except ImportError:
    VISUALIZER_AVAILABLE = False
from path_planning_gui import PathPlanningGUI


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
            'g': {'name': 'Gripper', 'min': 30, 'max': 80, 'current': 80}
        }
        
        # Target angles (from sliders)
        self.target_angles = {k: v['current'] for k, v in self.servos.items()}
        
        # Random angles storage
        self.random_angles = None
        
        # Track if we're currently sending commands
        self.sending_command = False
        
        # 3D Visualizer
        self.visualizer = None
        
        # Path Planning window
        self.path_planner_window = None
        
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
        self.port_entry.insert(0, "COM4" if tk.sys.platform.startswith('win') else "/dev/serial0")
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
        button_frame = ttk.LabelFrame(main_frame, text="Control", padding="10")
        button_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.home_btn = ttk.Button(button_frame, text="Home Position", 
                                   command=self.go_home, state='disabled')
        self.home_btn.grid(row=0, column=0, padx=5)
        
        self.refresh_btn = ttk.Button(button_frame, text="Refresh Angles", 
                                     command=self.refresh_angles, state='disabled')
        self.refresh_btn.grid(row=0, column=1, padx=5)
        
        # 3D Visualization button
        self.viz_btn = ttk.Button(button_frame, text="Open 3D View", 
                                 command=self.open_visualizer, state='normal')
        self.viz_btn.grid(row=0, column=2, padx=5)
        
        # Path Planning button
        self.path_btn = ttk.Button(button_frame, text="Path Planning", 
                                   command=self.open_path_planner, state='disabled')
        self.path_btn.grid(row=0, column=3, padx=5)
        
        # Smooth motion checkbox
        self.smooth_var = tk.BooleanVar(value=True)
        smooth_check = ttk.Checkbutton(button_frame, text="Smooth Motion (1° steps)", 
                                      variable=self.smooth_var)
        smooth_check.grid(row=0, column=4, padx=20)
        
        # Random movement buttons
        random_frame = ttk.LabelFrame(main_frame, text="Random Movement", padding="10")
        random_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.gen_random_btn = ttk.Button(random_frame, text="Generate Random Angles", 
                                        command=self.generate_random_angles, state='disabled')
        self.gen_random_btn.grid(row=0, column=0, padx=5)
        
        self.exec_random_btn = ttk.Button(random_frame, text="Execute Random Angles", 
                                         command=self.execute_random_angles, state='disabled')
        self.exec_random_btn.grid(row=0, column=1, padx=5)
        
        self.random_angles_label = ttk.Label(random_frame, text="No random angles generated", 
                                            font=('Arial', 9), foreground="gray")
        self.random_angles_label.grid(row=0, column=2, padx=20)
        
        # Position display frame
        pos_frame = ttk.LabelFrame(main_frame, text="End Effector Position (cm)", padding="10")
        pos_frame.grid(row=5, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # X, Y, Z labels
        self.x_label = ttk.Label(pos_frame, text="X: ---", font=('Arial', 11, 'bold'), width=15)
        self.x_label.grid(row=0, column=0, padx=10)
        
        self.y_label = ttk.Label(pos_frame, text="Y: ---", font=('Arial', 11, 'bold'), width=15)
        self.y_label.grid(row=0, column=1, padx=10)
        
        self.z_label = ttk.Label(pos_frame, text="Z: ---", font=('Arial', 11, 'bold'), width=15)
        self.z_label.grid(row=0, column=2, padx=10)
        
        # Inverse Kinematics frame
        ik_frame = ttk.LabelFrame(main_frame, text="Move to Position (Inverse Kinematics)", padding="10")
        ik_frame.grid(row=6, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # X input
        ttk.Label(ik_frame, text="X (cm):").grid(row=0, column=0, padx=5)
        self.ik_x_entry = ttk.Entry(ik_frame, width=10)
        self.ik_x_entry.insert(0, "0")
        self.ik_x_entry.grid(row=0, column=1, padx=5)
        
        # Y input
        ttk.Label(ik_frame, text="Y (cm):").grid(row=0, column=2, padx=5)
        self.ik_y_entry = ttk.Entry(ik_frame, width=10)
        self.ik_y_entry.insert(0, "20")
        self.ik_y_entry.grid(row=0, column=3, padx=5)
        
        # Z input
        ttk.Label(ik_frame, text="Z (cm):").grid(row=0, column=4, padx=5)
        self.ik_z_entry = ttk.Entry(ik_frame, width=10)
        self.ik_z_entry.insert(0, "40")
        self.ik_z_entry.grid(row=0, column=5, padx=5)
        
        # Move button
        self.move_to_pos_btn = ttk.Button(ik_frame, text="Move to Position", 
                                         command=self.move_to_position, state='disabled')
        self.move_to_pos_btn.grid(row=0, column=6, padx=10)
        
        # Copy current position button
        self.copy_pos_btn = ttk.Button(ik_frame, text="← Copy Current", 
                                      command=self.copy_current_position, state='disabled')
        self.copy_pos_btn.grid(row=0, column=7, padx=5)
        
        # Status bar
        status_bar = ttk.Frame(main_frame, relief=tk.SUNKEN)
        status_bar.grid(row=7, column=0, sticky=(tk.W, tk.E))
        
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
                
                # Update 3D visualizer if open
                self.update_visualizer()
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
                
                # Enable random movement buttons
                self.gen_random_btn.config(state='normal')
                
                # Enable IK buttons
                self.move_to_pos_btn.config(state='normal')
                self.copy_pos_btn.config(state='normal')
                
                # Enable path planning button
                self.path_btn.config(state='normal')
                
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
        self.gen_random_btn.config(state='disabled')
        self.exec_random_btn.config(state='disabled')
        self.move_to_pos_btn.config(state='disabled')
        self.copy_pos_btn.config(state='disabled')
        self.path_btn.config(state='disabled')
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
                home_angles = {'b': 90, 's': 130, 'e': 180, 'w': 180, 't': 180, 'g': 80}
                for servo_id, angle in home_angles.items():
                    self.root.after(0, lambda s=servo_id, a=angle: self.sliders[s].set(a))
                
                # Update 3D visualizer if open
                self.update_visualizer()
                
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
            
            # Update position display
            x, y, z = self.robot.get_end_effector_position()
            self.update_position_display(x, y, z)
            
            self.update_info("Angles refreshed")
        except Exception as e:
            self.update_info(f"Error: {e}")
    
    def generate_random_angles(self):
        """Generate random angles for all servos"""
        if not self.connected:
            return
        
        # Generate random angles within safe ranges
        self.random_angles = {
            'b': random.randint(0, 180),
            's': random.randint(0, 180),
            'e': random.randint(0, 180),
            'w': random.randint(0, 180),
            't': random.randint(0, 180),
            'g': random.randint(0, 80)
        }
        
        # Update label to show generated angles
        angles_str = f"b:{self.random_angles['b']}° s:{self.random_angles['s']}° e:{self.random_angles['e']}° w:{self.random_angles['w']}° t:{self.random_angles['t']}° g:{self.random_angles['g']}°"
        self.random_angles_label.config(text=angles_str, foreground="blue")
        
        # Enable execute button
        self.exec_random_btn.config(state='normal')
        
        self.update_info("Random angles generated. Click 'Execute' to move.")
    
    def execute_random_angles(self):
        """Execute the generated random angles"""
        if not self.connected or self.random_angles is None:
            return
        
        def execute():
            try:
                self.update_info("Executing random angles...")
                smooth = self.smooth_var.get()
                self.robot.send_command(self.random_angles, smooth=smooth, display_progress=False)
                
                # Update sliders to reflect new position
                for servo_id, angle in self.random_angles.items():
                    self.root.after(0, lambda s=servo_id, a=angle: self.sliders[s].set(a))
                
                # Update 3D visualizer if open
                self.update_visualizer()
                
                self.update_info("Random movement complete!")
                
                # Clear random angles after execution
                self.random_angles = None
                self.root.after(0, lambda: self.random_angles_label.config(
                    text="No random angles generated", foreground="gray"))
                self.root.after(0, lambda: self.exec_random_btn.config(state='disabled'))
                
            except Exception as e:
                self.update_info(f"Error: {e}")
        
        threading.Thread(target=execute, daemon=True).start()
    
    def move_to_position(self):
        """Move to position using inverse kinematics"""
        if not self.connected:
            return
        
        try:
            x = float(self.ik_x_entry.get())
            y = float(self.ik_y_entry.get())
            z = float(self.ik_z_entry.get())
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numeric values for X, Y, Z")
            return
        
        def move():
            try:
                self.update_info(f"Moving to position ({x:.2f}, {y:.2f}, {z:.2f})...")
                smooth = self.smooth_var.get()
                success = self.robot.move_to_position(x, y, z, smooth=smooth, display_progress=False)
                
                if success:
                    self.update_info(f"Successfully moved to ({x:.2f}, {y:.2f}, {z:.2f})")
                else:
                    self.update_info(f"Moved to ({x:.2f}, {y:.2f}, {z:.2f}) with warning (high error)")
                
                # Update sliders to match new angles
                current_angles = self.robot.get_current_angles()
                for servo_id, angle in current_angles.items():
                    if servo_id in self.sliders:
                        self.root.after(0, lambda s=servo_id, a=angle: self.sliders[s].set(a))
                
                # Update 3D visualizer if open
                self.update_visualizer()
                
            except Exception as e:
                self.update_info(f"Error: {e}")
                messagebox.showerror("Error", f"Failed to move to position: {e}")
        
        threading.Thread(target=move, daemon=True).start()
    
    def copy_current_position(self):
        """Copy current end effector position to IK input fields"""
        if not self.connected:
            return
        
        try:
            x, y, z = self.robot.get_end_effector_position()
            self.ik_x_entry.delete(0, tk.END)
            self.ik_x_entry.insert(0, f"{x:.2f}")
            self.ik_y_entry.delete(0, tk.END)
            self.ik_y_entry.insert(0, f"{y:.2f}")
            self.ik_z_entry.delete(0, tk.END)
            self.ik_z_entry.insert(0, f"{z:.2f}")
            self.update_info(f"Copied current position: ({x:.2f}, {y:.2f}, {z:.2f})")
        except Exception as e:
            self.update_info(f"Error: {e}")
    
    def update_loop(self):
        """Background thread to periodically update angles and position"""
        while self.running and self.connected:
            time.sleep(0.5)  # Update every 0.5 seconds
            if self.connected and not self.sending_command:
                try:
                    current_angles = self.robot.get_current_angles()
                    # Update labels only (not sliders to avoid interrupting user)
                    for servo_id, angle in current_angles.items():
                        if servo_id in self.angle_labels:
                            self.root.after(0, lambda s=servo_id, a=angle: 
                                          self.angle_labels[s].config(text=f"{a}°"))
                    
                    # Update end effector position
                    x, y, z = self.robot.get_end_effector_position()
                    self.root.after(0, lambda: self.update_position_display(x, y, z))
                    
                    # Update 3D visualizer if open
                    self.root.after(0, self.update_visualizer)
                except:
                    pass
    
    def update_position_display(self, x, y, z):
        """Update the position display labels"""
        self.x_label.config(text=f"X: {x:7.2f}")
        self.y_label.config(text=f"Y: {y:7.2f}")
        self.z_label.config(text=f"Z: {z:7.2f}")
    
    def update_info(self, message):
        """Update info label"""
        self.root.after(0, lambda: self.info_label.config(text=message))
    
    def open_visualizer(self):
        """Open 3D visualization window"""
        if not VISUALIZER_AVAILABLE:
            messagebox.showinfo("Not Available", "3D Visualizer module (visualizer_gui) is not installed.")
            return
        if self.visualizer is None or not self.visualizer.is_open:
            self.visualizer = RobotVisualizerWindow(parent=self.root)
            # Update with current angles
            if self.connected:
                angles = self.robot.get_current_angles()
            else:
                angles = {'b': 0, 's': 0, 'e': 0, 'w': 0}
            self.visualizer.update_arm(angles['b'], angles['s'], angles['e'], angles['w'])
            self.update_info("3D Visualizer opened")
        else:
            # Bring existing window to front
            self.visualizer.window.lift()
            self.visualizer.window.focus_force()
    
    def update_visualizer(self):
        """Update 3D visualizer with current angles"""
        if self.visualizer is not None and self.visualizer.is_open:
            if self.connected:
                angles = self.robot.get_current_angles()
                self.visualizer.update_arm(angles['b'], angles['s'], angles['e'], angles['w'])
    
    def open_path_planner(self):
        """Open path planning window"""
        if self.path_planner_window is None:
            # Create new window
            planner_root = tk.Toplevel(self.root)
            self.path_planner_window = PathPlanningGUI(planner_root, robot_controller=self.robot)
            
            # Handle window close
            def on_close():
                self.path_planner_window = None
                planner_root.destroy()
            
            planner_root.protocol("WM_DELETE_WINDOW", on_close)
            self.update_info("Path Planning window opened")
        else:
            # Bring existing window to front
            messagebox.showinfo("Path Planner", "Path planning window is already open")
    
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
