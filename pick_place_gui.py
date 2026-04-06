#!/usr/bin/env python3
"""
Pick-and-Place GUI for Prime-Arm Robot
Simple, focused interface for pick-and-place operations
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
from raspberry_pi_controller import RobotArmController


class PickPlaceGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Prime-Arm Pick & Place")
        self.root.geometry("600x750")
        
        self.robot = None
        self.executing = False
        
        self.create_widgets()
        self.connect_robot()
    
    def create_widgets(self):
        """Create GUI widgets"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title = ttk.Label(main_frame, text="🤖 Prime-Arm Pick & Place", 
                         font=('Arial', 18, 'bold'))
        title.pack(pady=(0, 15))
        
        # Connection Status
        status_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        status_frame.pack(fill=tk.X, pady=(0, 15))
        
        self.connection_label = ttk.Label(status_frame, text="⚫ Disconnected", 
                                         font=('Arial', 10), foreground='red')
        self.connection_label.pack(side=tk.LEFT)
        
        ttk.Button(status_frame, text="Reconnect", 
                  command=self.connect_robot).pack(side=tk.RIGHT, padx=5)
        
        self.current_pos_label = ttk.Label(status_frame, text="Current: --", 
                                          font=('Arial', 9), foreground='gray')
        self.current_pos_label.pack(side=tk.RIGHT, padx=15)
        
        # Pick Position
        pick_frame = ttk.LabelFrame(main_frame, text="📍 Pick Position", padding="10")
        pick_frame.pack(fill=tk.X, pady=(0, 10))
        
        pick_inputs = ttk.Frame(pick_frame)
        pick_inputs.pack(fill=tk.X)
        
        ttk.Label(pick_inputs, text="X:", font=('Arial', 10)).grid(row=0, column=0, padx=5, sticky='e')
        self.pick_x = ttk.Entry(pick_inputs, width=10, font=('Arial', 10))
        self.pick_x.insert(0, "-10")
        self.pick_x.grid(row=0, column=1, padx=5)
        
        ttk.Label(pick_inputs, text="Y:", font=('Arial', 10)).grid(row=0, column=2, padx=5, sticky='e')
        self.pick_y = ttk.Entry(pick_inputs, width=10, font=('Arial', 10))
        self.pick_y.insert(0, "10")
        self.pick_y.grid(row=0, column=3, padx=5)
        
        ttk.Label(pick_inputs, text="Z:", font=('Arial', 10)).grid(row=0, column=4, padx=5, sticky='e')
        self.pick_z = ttk.Entry(pick_inputs, width=10, font=('Arial', 10))
        self.pick_z.insert(0, "25")
        self.pick_z.grid(row=0, column=5, padx=5)
        
        ttk.Label(pick_inputs, text="cm", font=('Arial', 9), foreground='gray').grid(row=0, column=6, padx=5)
        
        pick_buttons = ttk.Frame(pick_frame)
        pick_buttons.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Button(pick_buttons, text="Use Current Position", 
                  command=self.use_current_for_pick).pack(side=tk.LEFT)
        ttk.Button(pick_buttons, text="Move to Pick", 
                  command=self.move_to_pick).pack(side=tk.LEFT, padx=5)
        
        # Place Position
        place_frame = ttk.LabelFrame(main_frame, text="📍 Place Position", padding="10")
        place_frame.pack(fill=tk.X, pady=(0, 10))
        
        place_inputs = ttk.Frame(place_frame)
        place_inputs.pack(fill=tk.X)
        
        ttk.Label(place_inputs, text="X:", font=('Arial', 10)).grid(row=0, column=0, padx=5, sticky='e')
        self.place_x = ttk.Entry(place_inputs, width=10, font=('Arial', 10))
        self.place_x.insert(0, "10")
        self.place_x.grid(row=0, column=1, padx=5)
        
        ttk.Label(place_inputs, text="Y:", font=('Arial', 10)).grid(row=0, column=2, padx=5, sticky='e')
        self.place_y = ttk.Entry(place_inputs, width=10, font=('Arial', 10))
        self.place_y.insert(0, "10")
        self.place_y.grid(row=0, column=3, padx=5)
        
        ttk.Label(place_inputs, text="Z:", font=('Arial', 10)).grid(row=0, column=4, padx=5, sticky='e')
        self.place_z = ttk.Entry(place_inputs, width=10, font=('Arial', 10))
        self.place_z.insert(0, "25")
        self.place_z.grid(row=0, column=5, padx=5)
        
        ttk.Label(place_inputs, text="cm", font=('Arial', 9), foreground='gray').grid(row=0, column=6, padx=5)
        
        place_buttons = ttk.Frame(place_frame)
        place_buttons.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Button(place_buttons, text="Use Current Position", 
                  command=self.use_current_for_place).pack(side=tk.LEFT)
        ttk.Button(place_buttons, text="Move to Place", 
                  command=self.move_to_place).pack(side=tk.LEFT, padx=5)
        
        # Settings
        settings_frame = ttk.LabelFrame(main_frame, text="⚙️ Settings", padding="10")
        settings_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Gripper Settings
        gripper_frame = ttk.Frame(settings_frame)
        gripper_frame.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(gripper_frame, text="Gripper Open:", font=('Arial', 10)).grid(row=0, column=0, padx=5, sticky='e')
        self.gripper_open = ttk.Entry(gripper_frame, width=8, font=('Arial', 10))
        self.gripper_open.insert(0, "30")
        self.gripper_open.grid(row=0, column=1, padx=5)
        ttk.Label(gripper_frame, text="°", font=('Arial', 9)).grid(row=0, column=2, sticky='w')
        
        ttk.Label(gripper_frame, text="Gripper Close:", font=('Arial', 10)).grid(row=0, column=3, padx=(20, 5), sticky='e')
        self.gripper_close = ttk.Entry(gripper_frame, width=8, font=('Arial', 10))
        self.gripper_close.insert(0, "80")
        self.gripper_close.grid(row=0, column=4, padx=5)
        ttk.Label(gripper_frame, text="°", font=('Arial', 9)).grid(row=0, column=5, sticky='w')
        
        # Safety Margins
        safety_frame = ttk.Frame(settings_frame)
        safety_frame.pack(fill=tk.X, pady=(5, 5))
        
        ttk.Label(safety_frame, text="Approach Height:", font=('Arial', 10)).grid(row=0, column=0, padx=5, sticky='e')
        self.approach_height = ttk.Entry(safety_frame, width=8, font=('Arial', 10))
        self.approach_height.insert(0, "10")
        self.approach_height.grid(row=0, column=1, padx=5)
        ttk.Label(safety_frame, text="cm", font=('Arial', 9)).grid(row=0, column=2, sticky='w')
        
        ttk.Label(safety_frame, text="Retract Height:", font=('Arial', 10)).grid(row=0, column=3, padx=(20, 5), sticky='e')
        self.retract_height = ttk.Entry(safety_frame, width=8, font=('Arial', 10))
        self.retract_height.insert(0, "10")
        self.retract_height.grid(row=0, column=4, padx=5)
        ttk.Label(safety_frame, text="cm", font=('Arial', 9)).grid(row=0, column=5, sticky='w')
        
        # Interpolation Method
        interp_frame = ttk.Frame(settings_frame)
        interp_frame.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Label(interp_frame, text="Motion:", font=('Arial', 10)).pack(side=tk.LEFT, padx=5)
        
        self.interp_var = tk.StringVar(value="cubic")
        ttk.Radiobutton(interp_frame, text="Linear", variable=self.interp_var, 
                       value="linear").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(interp_frame, text="Cubic (smooth)", variable=self.interp_var, 
                       value="cubic").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(interp_frame, text="Quintic (smoothest)", variable=self.interp_var, 
                       value="quintic").pack(side=tk.LEFT, padx=5)
        
        # Speed
        speed_frame = ttk.Frame(settings_frame)
        speed_frame.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Label(speed_frame, text="Speed:", font=('Arial', 10)).pack(side=tk.LEFT, padx=5)
        self.speed = ttk.Scale(speed_frame, from_=0.005, to=0.2, orient=tk.HORIZONTAL, length=250)
        self.speed.set(0.02)
        self.speed.pack(side=tk.LEFT, padx=5)
        self.speed_label = ttk.Label(speed_frame, text="0.020 s", font=('Arial', 9))
        self.speed_label.pack(side=tk.LEFT, padx=5)
        self.speed.config(command=self.update_speed_label)
        
        # Execute Button
        execute_frame = ttk.Frame(main_frame)
        execute_frame.pack(fill=tk.X, pady=(10, 10))
        
        self.execute_btn = ttk.Button(execute_frame, text="▶️ Execute Pick & Place", 
                                     command=self.execute_pick_place,
                                     style='Accent.TButton')
        self.execute_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        self.stop_btn = ttk.Button(execute_frame, text="⏹️ Stop", 
                                   command=self.stop_execution,
                                   state='disabled')
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Quick Actions
        actions_frame = ttk.LabelFrame(main_frame, text="Quick Actions", padding="10")
        actions_frame.pack(fill=tk.X, pady=(0, 10))
        
        actions_grid = ttk.Frame(actions_frame)
        actions_grid.pack()
        
        ttk.Button(actions_grid, text="🏠 Home Position", 
                  command=self.go_home, width=20).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(actions_grid, text="📋 Test Gripper", 
                  command=self.test_gripper, width=20).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(actions_grid, text="🔄 Refresh Position", 
                  command=self.update_current_position, width=20).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(actions_grid, text="📊 Show Path Preview", 
                  command=self.show_path_preview, width=20).grid(row=1, column=1, padx=5, pady=5)
        
        # Status Bar
        status_bar_frame = ttk.Frame(main_frame, relief=tk.SUNKEN)
        status_bar_frame.pack(fill=tk.X, pady=(5, 0))
        
        self.status_label = ttk.Label(status_bar_frame, text="Ready", 
                                     font=('Arial', 9), foreground='green')
        self.status_label.pack(side=tk.LEFT, padx=5, pady=3)
    
    def update_speed_label(self, value):
        """Update speed label when slider moves"""
        self.speed_label.config(text=f"{float(value):.3f} s")
    
    def connect_robot(self):
        """Connect to robot"""
        if self.robot:
            self.robot.disconnect()
        
        try:
            self.robot = RobotArmController(port='COM4')
            if self.robot.connect():
                self.connection_label.config(text="🟢 Connected", foreground='green')
                self.status_label.config(text="Robot connected successfully", foreground='green')
                self.update_current_position()
            else:
                self.connection_label.config(text="🔴 Connection Failed", foreground='red')
                self.status_label.config(text="Failed to connect to robot", foreground='red')
                self.robot = None
        except Exception as e:
            self.connection_label.config(text="🔴 Error", foreground='red')
            self.status_label.config(text=f"Error: {e}", foreground='red')
            self.robot = None
    
    def update_current_position(self):
        """Update current position display"""
        if not self.robot:
            self.current_pos_label.config(text="Current: Not connected")
            return
        
        try:
            x, y, z = self.robot.get_end_effector_position()
            self.current_pos_label.config(text=f"Current: ({x:.1f}, {y:.1f}, {z:.1f}) cm")
        except Exception as e:
            self.current_pos_label.config(text=f"Current: Error - {e}")
    
    def use_current_for_pick(self):
        """Use current robot position as pick position"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        try:
            x, y, z = self.robot.get_end_effector_position()
            self.pick_x.delete(0, tk.END)
            self.pick_x.insert(0, f"{x:.1f}")
            self.pick_y.delete(0, tk.END)
            self.pick_y.insert(0, f"{y:.1f}")
            self.pick_z.delete(0, tk.END)
            self.pick_z.insert(0, f"{z:.1f}")
            self.status_label.config(text=f"Pick position set to: ({x:.1f}, {y:.1f}, {z:.1f})", foreground='blue')
        except Exception as e:
            messagebox.showerror("Error", f"Failed to get position: {e}")
    
    def use_current_for_place(self):
        """Use current robot position as place position"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        try:
            x, y, z = self.robot.get_end_effector_position()
            self.place_x.delete(0, tk.END)
            self.place_x.insert(0, f"{x:.1f}")
            self.place_y.delete(0, tk.END)
            self.place_y.insert(0, f"{y:.1f}")
            self.place_z.delete(0, tk.END)
            self.place_z.insert(0, f"{z:.1f}")
            self.status_label.config(text=f"Place position set to: ({x:.1f}, {y:.1f}, {z:.1f})", foreground='blue')
        except Exception as e:
            messagebox.showerror("Error", f"Failed to get position: {e}")
    
    def move_to_pick(self):
        """Move robot to pick position"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        try:
            x = float(self.pick_x.get())
            y = float(self.pick_y.get())
            z = float(self.pick_z.get())
            
            def move():
                self.status_label.config(text=f"Moving to pick position...", foreground='blue')
                self.robot.move_to_position(x, y, z, smooth=True, display_progress=False)
                self.update_current_position()
                self.status_label.config(text=f"Arrived at pick position", foreground='green')
            
            threading.Thread(target=move, daemon=True).start()
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid coordinates")
    
    def move_to_place(self):
        """Move robot to place position"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        try:
            x = float(self.place_x.get())
            y = float(self.place_y.get())
            z = float(self.place_z.get())
            
            def move():
                self.status_label.config(text=f"Moving to place position...", foreground='blue')
                self.robot.move_to_position(x, y, z, smooth=True, display_progress=False)
                self.update_current_position()
                self.status_label.config(text=f"Arrived at place position", foreground='green')
            
            threading.Thread(target=move, daemon=True).start()
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid coordinates")
    
    def execute_pick_place(self):
        """Execute pick and place operation"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        try:
            # Get positions
            pick_pos = (float(self.pick_x.get()), float(self.pick_y.get()), float(self.pick_z.get()))
            place_pos = (float(self.place_x.get()), float(self.place_y.get()), float(self.place_z.get()))
            
            # Get settings
            gripper_open_angle = max(30, min(80, int(self.gripper_open.get())))
            gripper_close_angle = max(30, min(80, int(self.gripper_close.get())))
            approach_h = float(self.approach_height.get())
            retract_h = float(self.retract_height.get())
            dt = float(self.speed.get())
            
            def execute():
                try:
                    self.executing = True
                    self.execute_btn.config(state='disabled')
                    self.stop_btn.config(state='normal')
                    
                    self.status_label.config(text="Executing pick-and-place…", foreground='blue')
                    
                    # Execute with joint-space interpolation (no path planning needed)
                    self.robot.execute_pick_and_place(
                        pick_pos=pick_pos,
                        place_pos=place_pos,
                        approach_height=approach_h,
                        retract_height=retract_h,
                        gripper_open=gripper_open_angle,
                        gripper_close=gripper_close_angle,
                        dt=dt,
                        display_progress=True
                    )
                    
                    if self.executing:
                        self.status_label.config(text="✓ Pick-and-place completed successfully!", foreground='green')
                        self.update_current_position()
                    else:
                        self.status_label.config(text="Operation stopped", foreground='orange')
                
                except Exception as e:
                    self.status_label.config(text=f"Error: {e}", foreground='red')
                    messagebox.showerror("Execution Error", f"Failed: {e}")
                finally:
                    self.executing = False
                    self.execute_btn.config(state='normal')
                    self.stop_btn.config(state='disabled')
            
            threading.Thread(target=execute, daemon=True).start()
            
        except ValueError:
            messagebox.showerror("Invalid Input", "Please check all input values")
    
    def stop_execution(self):
        """Stop pick-and-place execution"""
        self.executing = False
        self.status_label.config(text="Stopping...", foreground='orange')
    
    def go_home(self):
        """Move robot to home position"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        def move_home():
            self.status_label.config(text="Moving to home position...", foreground='blue')
            self.robot.home_position(smooth=True, display_progress=False)
            self.update_current_position()
            self.status_label.config(text="Home position reached", foreground='green')
        
        threading.Thread(target=move_home, daemon=True).start()
    
    def test_gripper(self):
        """Test gripper open/close"""
        if not self.robot:
            messagebox.showwarning("No Robot", "Robot not connected")
            return
        
        try:
            open_angle = max(30, min(80, int(self.gripper_open.get())))
            close_angle = max(30, min(80, int(self.gripper_close.get())))
            
            def test():
                self.status_label.config(text="Testing gripper...", foreground='blue')
                
                # Close
                self.robot.send_command({'g': close_angle}, smooth=True, display_progress=False)
                import time
                time.sleep(1)
                
                # Open
                self.robot.send_command({'g': open_angle}, smooth=True, display_progress=False)
                time.sleep(1)
                
                self.status_label.config(text="Gripper test complete", foreground='green')
            
            threading.Thread(target=test, daemon=True).start()
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid gripper angles")
    
    def show_path_preview(self):
        """Show IK preview for the 5 key positions"""
        try:
            pick_pos = (float(self.pick_x.get()), float(self.pick_y.get()), float(self.pick_z.get()))
            place_pos = (float(self.place_x.get()), float(self.place_y.get()), float(self.place_z.get()))
            approach_h = float(self.approach_height.get())
            retract_h = float(self.retract_height.get())
            dt = float(self.speed.get())
            steps = 25   # default steps_per_segment

            px, py, pz = pick_pos
            lx, ly, lz = place_pos

            from inverse_kinematics_servo import calculate_ik
            guess = None
            if self.robot:
                cur = self.robot.get_current_angles()
                guess = {k: cur[k] for k in ('b', 's', 'e', 'w')}

            lines = ["Pick & Place IK Preview", "="*40]
            for label, pos in [
                ("above_pick",  (px, py, pz + approach_h)),
                ("at_pick",     (px, py, pz)),
                ("above_place", (lx, ly, lz + retract_h)),
                ("at_place",    (lx, ly, lz)),
            ]:
                if guess is not None:
                    angles, ok, err = calculate_ik(*pos, guess)
                    st = 'OK' if ok else '!'
                    lines.append(f"{label:14s}: b={angles['b']:3d} s={angles['s']:3d} "
                                 f"e={angles['e']:3d} w={angles['w']:3d}  [{st} err={err:.2f}cm]")
                    guess = angles
                else:
                    lines.append(f"{label:14s}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

            total = 6 * steps
            lines += ["", f"Steps/segment: {steps}  Total commands: {total}",
                      f"Estimated time: {total * dt:.1f} s"]

            messagebox.showinfo("Path Preview", "\n".join(lines))

        except ValueError:
            messagebox.showerror("Invalid Input", "Please check all input values")
        except Exception as e:
            messagebox.showerror("Preview Error", f"Failed: {e}")


def main():
    root = tk.Tk()
    app = PickPlaceGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
