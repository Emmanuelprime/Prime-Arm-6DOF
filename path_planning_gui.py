#!/usr/bin/env python3
"""
Path Planning GUI for Prime-Arm Robot
Plan and execute multi-waypoint paths
"""

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import threading
import random
from path_planning import PathPlanner
from raspberry_pi_controller import RobotArmController


class PathPlanningGUI:
    def __init__(self, root, robot_controller=None):
        self.root = root
        self.root.title("Path Planning - Prime-Arm")
        self.root.geometry("900x700")
        
        self.robot = robot_controller
        self.planner = PathPlanner()
        
        # Waypoints list
        self.waypoints = []
        
        # Planned paths
        self.cartesian_path = None
        self.joint_path = None
        self.is_pick_place = False
        
        self.create_widgets()
    
    def create_widgets(self):
        """Create GUI widgets"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title = ttk.Label(main_frame, text="Path Planning & Execution", 
                         font=('Arial', 16, 'bold'))
        title.pack(pady=(0, 10))
        
        # Waypoints Frame
        waypoints_frame = ttk.LabelFrame(main_frame, text="Waypoints", padding="10")
        waypoints_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # Waypoint list
        list_frame = ttk.Frame(waypoints_frame)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.waypoints_listbox = tk.Listbox(list_frame, height=10, 
                                             font=('Courier', 10),
                                             yscrollcommand=scrollbar.set)
        self.waypoints_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.waypoints_listbox.yview)
        
        # Waypoint controls
        wp_control_frame = ttk.Frame(waypoints_frame)
        wp_control_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Add waypoint inputs
        ttk.Label(wp_control_frame, text="X:").grid(row=0, column=0, padx=5)
        self.wp_x_entry = ttk.Entry(wp_control_frame, width=8)
        self.wp_x_entry.insert(0, "0")
        self.wp_x_entry.grid(row=0, column=1, padx=5)
        
        ttk.Label(wp_control_frame, text="Y:").grid(row=0, column=2, padx=5)
        self.wp_y_entry = ttk.Entry(wp_control_frame, width=8)
        self.wp_y_entry.insert(0, "20")
        self.wp_y_entry.grid(row=0, column=3, padx=5)
        
        ttk.Label(wp_control_frame, text="Z:").grid(row=0, column=4, padx=5)
        self.wp_z_entry = ttk.Entry(wp_control_frame, width=8)
        self.wp_z_entry.insert(0, "40")
        self.wp_z_entry.grid(row=0, column=5, padx=5)
        
        ttk.Button(wp_control_frame, text="Add Waypoint", 
                  command=self.add_waypoint).grid(row=0, column=6, padx=10)
        ttk.Button(wp_control_frame, text="Use Current Position", 
                  command=self.add_current_position).grid(row=0, column=7, padx=5)
        
        # List control buttons
        wp_button_frame = ttk.Frame(waypoints_frame)
        wp_button_frame.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Button(wp_button_frame, text="Remove Selected", 
                  command=self.remove_waypoint).pack(side=tk.LEFT, padx=5)
        ttk.Button(wp_button_frame, text="Clear All", 
                  command=self.clear_waypoints).pack(side=tk.LEFT, padx=5)
        ttk.Button(wp_button_frame, text="Move Up", 
                  command=self.move_waypoint_up).pack(side=tk.LEFT, padx=5)
        ttk.Button(wp_button_frame, text="Move Down", 
                  command=self.move_waypoint_down).pack(side=tk.LEFT, padx=5)
        ttk.Button(wp_button_frame, text="Generate Random Waypoints", 
                  command=self.generate_random_waypoints).pack(side=tk.LEFT, padx=20)
        
        # Path Planning Frame
        planning_frame = ttk.LabelFrame(main_frame, text="Path Planning", padding="10")
        planning_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Path type selection
        type_frame = ttk.Frame(planning_frame)
        type_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(type_frame, text="Path Type:").pack(side=tk.LEFT, padx=5)
        self.path_type_var = tk.StringVar(value="straight")
        self.path_type_var.trace('w', self.on_path_type_change)
        ttk.Radiobutton(type_frame, text="Straight Line", variable=self.path_type_var, 
                       value="straight").pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(type_frame, text="Arc", variable=self.path_type_var, 
                       value="arc").pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(type_frame, text="Pick & Place", variable=self.path_type_var, 
                       value="pick_place").pack(side=tk.LEFT, padx=10)
        
        # Interpolation method selection
        interp_frame = ttk.Frame(planning_frame)
        interp_frame.pack(fill=tk.X, pady=(5, 10))
        
        ttk.Label(interp_frame, text="Interpolation:").pack(side=tk.LEFT, padx=5)
        self.interp_type_var = tk.StringVar(value="linear")
        ttk.Radiobutton(interp_frame, text="Linear", variable=self.interp_type_var, 
                       value="linear").pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(interp_frame, text="Cubic (smooth velocity)", variable=self.interp_type_var, 
                       value="cubic").pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(interp_frame, text="Quintic (smooth accel)", variable=self.interp_type_var, 
                       value="quintic").pack(side=tk.LEFT, padx=10)
        
        # Parameters frame
        params_frame = ttk.Frame(planning_frame)
        params_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Arc height (for arc mode)
        ttk.Label(params_frame, text="Arc Height (cm):").grid(row=0, column=0, padx=5, sticky='w')
        self.arc_height_entry = ttk.Entry(params_frame, width=8)
        self.arc_height_entry.insert(0, "10")
        self.arc_height_entry.grid(row=0, column=1, padx=5)
        
        # Approach height (for pick & place)
        ttk.Label(params_frame, text="Approach Height (cm):").grid(row=0, column=2, padx=5, sticky='w')
        self.approach_height_entry = ttk.Entry(params_frame, width=8)
        self.approach_height_entry.insert(0, "10")
        self.approach_height_entry.grid(row=0, column=3, padx=5)
        self.approach_height_entry.config(state='disabled')
        
        # Retract height
        ttk.Label(params_frame, text="Retract Height (cm):").grid(row=0, column=4, padx=5, sticky='w')
        self.retract_height_entry = ttk.Entry(params_frame, width=8)
        self.retract_height_entry.insert(0, "10")
        self.retract_height_entry.grid(row=0, column=5, padx=5)
        self.retract_height_entry.config(state='disabled')
        
        # Points per segment
        ttk.Label(params_frame, text="Points/Segment:").grid(row=1, column=0, padx=5, pady=(5,0), sticky='w')
        self.points_entry = ttk.Entry(params_frame, width=8)
        self.points_entry.insert(0, "10")
        self.points_entry.grid(row=1, column=1, padx=5, pady=(5,0))
        
        # Gripper settings for pick & place
        ttk.Label(params_frame, text="Gripper Open:").grid(row=1, column=2, padx=5, pady=(5,0), sticky='w')
        self.gripper_open_entry = ttk.Entry(params_frame, width=8)
        self.gripper_open_entry.insert(0, "30")
        self.gripper_open_entry.grid(row=1, column=3, padx=5, pady=(5,0))
        self.gripper_open_entry.config(state='disabled')
        
        ttk.Label(params_frame, text="Gripper Close:").grid(row=1, column=4, padx=5, pady=(5,0), sticky='w')
        self.gripper_close_entry = ttk.Entry(params_frame, width=8)
        self.gripper_close_entry.insert(0, "80")
        self.gripper_close_entry.grid(row=1, column=5, padx=5, pady=(5,0))
        self.gripper_close_entry.config(state='disabled')
        
        # Plan button
        plan_button_frame = ttk.Frame(planning_frame)
        plan_button_frame.pack(fill=tk.X)
        
        self.plan_btn = ttk.Button(plan_button_frame, text="Plan Path", 
                                   command=self.plan_path,
                                   style='Accent.TButton')
        self.plan_btn.pack(side=tk.LEFT, padx=5)
        
        self.path_info_label = ttk.Label(plan_button_frame, text="No path planned", 
                                         foreground="gray")
        self.path_info_label.pack(side=tk.LEFT, padx=20)
        
        # Execution Frame
        exec_frame = ttk.LabelFrame(main_frame, text="Path Execution", padding="10")
        exec_frame.pack(fill=tk.X, pady=(0, 10))
        
        exec_control_frame = ttk.Frame(exec_frame)
        exec_control_frame.pack(fill=tk.X)
        
        ttk.Label(exec_control_frame, text="Step delay (s):").pack(side=tk.LEFT, padx=5)
        self.dt_entry = ttk.Entry(exec_control_frame, width=8)
        self.dt_entry.insert(0, "0.015")
        self.dt_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(exec_control_frame, text="(Lower = faster motion)", 
                 font=('Arial', 8), foreground='gray').pack(side=tk.LEFT, padx=5)
        
        self.execute_btn = ttk.Button(exec_control_frame, text="Execute Path", 
                                      command=self.execute_path, state='disabled')
        self.execute_btn.pack(side=tk.LEFT, padx=20)
        
        self.stop_btn = ttk.Button(exec_control_frame, text="Stop", 
                                   command=self.stop_execution, state='disabled')
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        self.home_btn = ttk.Button(exec_control_frame, text="Home Position", 
                                   command=self.go_home)
        self.home_btn.pack(side=tk.LEFT, padx=20)
        
        # Status
        status_frame = ttk.Frame(main_frame, relief=tk.SUNKEN)
        status_frame.pack(fill=tk.X)
        
        self.status_label = ttk.Label(status_frame, text="Ready. Add waypoints to begin. Path uses smooth 1° stepping.", 
                                      font=('Arial', 9))
        self.status_label.pack(side=tk.LEFT, padx=5, pady=2)
        
        # Execution flag
        self.executing = False
    
    def add_waypoint(self):
        """Add waypoint from input fields"""
        try:
            x = float(self.wp_x_entry.get())
            y = float(self.wp_y_entry.get())
            z = float(self.wp_z_entry.get())
            
            # Check workspace limits
            if not self.planner.check_workspace_limits((x, y, z)):
                messagebox.showwarning("Out of Bounds", 
                                     f"Position ({x}, {y}, {z}) is outside workspace limits!")
                return
            
            self.waypoints.append((x, y, z))
            self.update_waypoint_list()
            self.status_label.config(text=f"Added waypoint: ({x:.1f}, {y:.1f}, {z:.1f})")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numeric values")
    
    def add_current_position(self):
        """Add current robot position as waypoint"""
        if self.robot is None:
            messagebox.showwarning("No Robot", "Robot controller not connected")
            return
        
        try:
            x, y, z = self.robot.get_end_effector_position()
            self.waypoints.append((x, y, z))
            self.update_waypoint_list()
            self.status_label.config(text=f"Added current position: ({x:.1f}, {y:.1f}, {z:.1f})")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to get current position: {e}")
    
    def remove_waypoint(self):
        """Remove selected waypoint"""
        selection = self.waypoints_listbox.curselection()
        if selection:
            idx = selection[0]
            self.waypoints.pop(idx)
            self.update_waypoint_list()
            self.status_label.config(text="Waypoint removed")
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints.clear()
        self.update_waypoint_list()
        self.cartesian_path = None
        self.is_pick_place = False
        self.path_info_label.config(text="No path planned", foreground="gray")
        self.execute_btn.config(state='disabled')
        self.status_label.config(text="All waypoints cleared")
    
    def move_waypoint_up(self):
        """Move selected waypoint up in list"""
        selection = self.waypoints_listbox.curselection()
        if selection and selection[0] > 0:
            idx = selection[0]
            self.waypoints[idx], self.waypoints[idx-1] = self.waypoints[idx-1], self.waypoints[idx]
            self.update_waypoint_list()
            self.waypoints_listbox.selection_set(idx-1)
    
    def move_waypoint_down(self):
        """Move selected waypoint down in list"""
        selection = self.waypoints_listbox.curselection()
        if selection and selection[0] < len(self.waypoints) - 1:
            idx = selection[0]
            self.waypoints[idx], self.waypoints[idx+1] = self.waypoints[idx+1], self.waypoints[idx]
            self.update_waypoint_list()
            self.waypoints_listbox.selection_set(idx+1)
    
    def update_waypoint_list(self):
        """Update waypoint listbox"""
        self.waypoints_listbox.delete(0, tk.END)
        for i, wp in enumerate(self.waypoints):
            self.waypoints_listbox.insert(tk.END, f"{i+1}. X:{wp[0]:6.1f}  Y:{wp[1]:6.1f}  Z:{wp[2]:6.1f}")
    
    def on_path_type_change(self, *args):
        """Handle path type selection change"""
        path_type = self.path_type_var.get()
        
        if path_type == "pick_place":
            # Enable pick & place parameters
            self.approach_height_entry.config(state='normal')
            self.retract_height_entry.config(state='normal')
            self.gripper_open_entry.config(state='normal')
            self.gripper_close_entry.config(state='normal')
        else:
            # Disable pick & place parameters
            self.approach_height_entry.config(state='disabled')
            self.retract_height_entry.config(state='disabled')
            self.gripper_open_entry.config(state='disabled')
            self.gripper_close_entry.config(state='disabled')
    
    def plan_path(self):
        """Plan path through waypoints"""
        if len(self.waypoints) < 2:
            messagebox.showwarning("Insufficient Waypoints", "Need at least 2 waypoints to plan a path")
            return
        
        try:
            points = int(self.points_entry.get())
            path_type = self.path_type_var.get()
            interp_type = self.interp_type_var.get()
            
            self.status_label.config(text="Planning path...")
            
            if path_type == "straight":
                # Use selected interpolation method
                if interp_type == "cubic":
                    self.cartesian_path = self.planner.plan_waypoints_cubic_spline(
                        self.waypoints, points, smooth_velocity=True
                    )
                elif interp_type == "quintic":
                    # For quintic with multiple waypoints, use cubic spline
                    # (quintic is best for single segment)
                    self.cartesian_path = self.planner.plan_waypoints_cubic_spline(
                        self.waypoints, points, smooth_velocity=True
                    )
                else:  # linear
                    self.cartesian_path = self.planner.plan_waypoints_cartesian(
                        self.waypoints, points
                    )
                self.is_pick_place = False
                
            elif path_type == "arc":
                if len(self.waypoints) != 2:
                    messagebox.showwarning("Arc Path", "Arc path requires exactly 2 waypoints")
                    return
                arc_height = float(self.arc_height_entry.get())
                self.cartesian_path = self.planner.plan_cartesian_arc(
                    self.waypoints[0], self.waypoints[1], arc_height, points
                )
                self.is_pick_place = False
                
            elif path_type == "pick_place":
                if len(self.waypoints) != 2:
                    messagebox.showwarning("Pick & Place", "Pick & Place requires exactly 2 waypoints (pick and place)")
                    return
                
                approach_height = float(self.approach_height_entry.get())
                retract_height = float(self.retract_height_entry.get())
                
                # Get current robot position for proper approach planning
                current_pos = None
                if self.robot is not None:
                    try:
                        current_pos = self.robot.get_end_effector_position()
                    except:
                        pass  # If fails, path will start at approach position
                
                # Use selected interpolation method for smooth pick-and-place
                phases, full_path = self.planner.plan_pick_and_place(
                    self.waypoints[0], self.waypoints[1],
                    current_pos=current_pos,
                    approach_height=approach_height,
                    retract_height=retract_height,
                    num_points_approach=5,
                    num_points_transfer=points,
                    interpolation=interp_type
                )
                
                # Store pick-and-place path with gripper commands
                self.cartesian_path = full_path
                self.is_pick_place = True
                
                # Verify pick position is in path
                pick_wp_nums = []
                for i, waypoint in enumerate(full_path):
                    x, y, z, _ = waypoint
                    if (abs(x - self.waypoints[0][0]) < 0.1 and 
                        abs(y - self.waypoints[0][1]) < 0.1 and 
                        abs(z - self.waypoints[0][2]) < 0.1):
                        pick_wp_nums.append(i+1)
                
                # Display phase information
                phase_info = []
                for phase_name, phase_path in phases.items():
                    phase_info.append(f"{phase_name}: {len(phase_path)} pts")
                
                pick_wp_str = f" | Pick at waypoints: {pick_wp_nums}" if pick_wp_nums else " | ⚠️ Pick position NOT in path!"
                
                # Add interpolation method to info
                interp_label = {"linear": "Linear", "cubic": "Cubic", "quintic": "Quintic"}
                interp_info = f" ({interp_label[interp_type]})"
                
                self.path_info_label.config(
                    text=f"Pick & Place planned: {len(full_path)} waypoints{pick_wp_str}{interp_info}",
                    foreground="blue"
                )
                self.execute_btn.config(state='normal')
                self.status_label.config(text=f"Pick & Place complete using {interp_label[interp_type]} interpolation")
                return
            
            # Compute path info for non-pick-place paths
            # Extract just positions for path length calculation
            if self.is_pick_place:
                positions = [(p[0], p[1], p[2]) for p in self.cartesian_path]
                path_length = self.planner.compute_path_length(positions)
            else:
                path_length = self.planner.compute_path_length(self.cartesian_path)
            
            # Add interpolation method to info
            interp_label = {"linear": "Linear", "cubic": "Cubic", "quintic": "Quintic"}
            interp_info = f" ({interp_label[interp_type]} interpolation)"
            
            self.path_info_label.config(
                text=f"Path planned: {len(self.cartesian_path)} points, {path_length:.1f} cm{interp_info}",
                foreground="blue"
            )
            self.execute_btn.config(state='normal')
            self.status_label.config(text=f"Path planning complete using {interp_label[interp_type]} interpolation")
            
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numeric values")
        except Exception as e:
            messagebox.showerror("Planning Error", f"Failed to plan path: {e}")
    
    def execute_path(self):
        """Execute planned path"""
        if self.cartesian_path is None:
            messagebox.showwarning("No Path", "Please plan a path first")
            return
        
        if self.robot is None:
            messagebox.showwarning("No Robot", "Robot controller not connected")
            return
        
        try:
            dt = float(self.dt_entry.get())
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid time step")
            return
        
        def execute():
            try:
                self.executing = True
                self.execute_btn.config(state='disabled')
                self.stop_btn.config(state='normal')
                self.plan_btn.config(state='disabled')
                
                if self.is_pick_place:
                    # Execute pick-and-place operation with gripper control
                    self.status_label.config(text="Executing pick-and-place operation...")
                    try:
                        gripper_open = int(self.gripper_open_entry.get())
                        gripper_close = int(self.gripper_close_entry.get())
                        # SAFETY: Clamp to safe range, where 30° is fully open
                        gripper_open = max(30, min(80, gripper_open))
                        gripper_close = max(30, min(80, gripper_close))
                    except:
                        gripper_open = 30
                        gripper_close = 80
                    
                    print("\n" + "="*70)
                    print("PICK-AND-PLACE EXECUTION")
                    print("="*70)
                    print(f"Total waypoints: {len(self.cartesian_path)}")
                    print(f"Pick position: {self.waypoints[0]}")
                    print(f"Place position: {self.waypoints[1]}")
                    print("="*70 + "\n")
                    
                    self.robot.execute_cartesian_pick_and_place(
                        self.cartesian_path,
                        dt=dt,
                        gripper_open=gripper_open,
                        gripper_close=gripper_close,
                        display_progress=True
                    )
                else:
                    # Execute regular Cartesian path
                    self.status_label.config(text="Executing path with smooth motion...")
                    self.robot.execute_cartesian_path(self.cartesian_path, dt=dt, display_progress=False)
                
                if self.executing:
                    self.status_label.config(text="Path execution complete!")
                else:
                    self.status_label.config(text="Path execution stopped")
                
            except Exception as e:
                self.status_label.config(text=f"Execution error: {e}")
                messagebox.showerror("Execution Error", f"Failed to execute path: {e}")
            finally:
                self.executing = False
                self.execute_btn.config(state='normal')
                self.stop_btn.config(state='disabled')
                self.plan_btn.config(state='normal')
        
        threading.Thread(target=execute, daemon=True).start()
    
    def stop_execution(self):
        """Stop path execution"""
        self.executing = False
        self.status_label.config(text="Stopping execution...")
    
    def generate_random_waypoints(self):
        """Generate random waypoints within workspace"""
        # Ask user how many waypoints
        num_points = tk.simpledialog.askinteger(
            "Random Waypoints",
            "How many random waypoints to generate?",
            initialvalue=5,
            minvalue=2,
            maxvalue=20
        )
        
        if num_points is None:
            return
        
        # Clear existing waypoints
        self.waypoints.clear()
        
        # Generate random waypoints
        for i in range(num_points):
            x = round(random.uniform(-30, 30), 1)
            y = round(random.uniform(15, 35), 1)
            z = round(random.uniform(30, 55), 1)
            self.waypoints.append((x, y, z))
        
        self.update_waypoint_list()
        self.status_label.config(text=f"Generated {num_points} random waypoints")
    
    def go_home(self):
        """Move robot to home position"""
        if self.robot is None:
            messagebox.showwarning("No Robot", "Robot controller not connected")
            return
        
        def move_home():
            try:
                self.status_label.config(text="Moving to home position...")
                self.home_btn.config(state='disabled')
                
                self.robot.home_position(smooth=True, display_progress=False)
                
                self.status_label.config(text="Home position reached")
            except Exception as e:
                self.status_label.config(text=f"Error: {e}")
                messagebox.showerror("Error", f"Failed to move home: {e}")
            finally:
                self.home_btn.config(state='normal')
        
        threading.Thread(target=move_home, daemon=True).start()


def main():
    """Standalone mode"""
    root = tk.Tk()
    
    # Try to connect to robot (optional)
    robot = None
    try:
        robot = RobotArmController(port='COM4' if tk.sys.platform.startswith('win') else '/dev/ttyACM0')
        if robot.connect():
            print("Robot connected")
        else:
            robot = None
    except:
        print("Running without robot connection")
    
    app = PathPlanningGUI(root, robot_controller=robot)
    root.mainloop()
    
    if robot:
        robot.disconnect()


if __name__ == "__main__":
    main()
