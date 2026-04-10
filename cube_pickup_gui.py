#!/usr/bin/env python3
"""
Cube Pick-and-Place GUI for Prime-Arm Robot
Configurable interface for picking and placing a box/cube.
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
from raspberry_pi_controller import RobotArmController


class CubePickupGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Prime-Arm — Cube Pick & Place")
        self.root.geometry("720x820")
        self.root.resizable(True, True)

        self.robot = None
        self.executing = False
        self._stop_requested = False
        self.planned_params = None   # set by _plan, consumed by _execute

        self._build_ui()
        self._connect_robot()

    # ------------------------------------------------------------------ UI --
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main, text="📦  Cube Pick & Place", font=('Arial', 18, 'bold')).pack(pady=(0, 8))

        # ── Connection bar ────────────────────────────────────────────────
        conn = ttk.LabelFrame(main, text="Connection", padding=8)
        conn.pack(fill=tk.X, pady=(0, 8))

        left = ttk.Frame(conn)
        left.pack(side=tk.LEFT, fill=tk.X, expand=True)

        ttk.Label(left, text="Port:").pack(side=tk.LEFT)
        import sys
        self.port_var = tk.StringVar(
            value='COM4' if sys.platform.startswith('win') else '/dev/serial0')
        ttk.Entry(left, textvariable=self.port_var, width=10).pack(side=tk.LEFT, padx=4)
        ttk.Button(left, text="Connect", command=self._connect_robot).pack(side=tk.LEFT, padx=4)

        self.conn_label = ttk.Label(conn, text="⚫ Disconnected", foreground='red',
                                    font=('Arial', 10, 'bold'))
        self.conn_label.pack(side=tk.RIGHT, padx=6)

        self.pos_label = ttk.Label(conn, text="Position: --", font=('Arial', 9), foreground='gray')
        self.pos_label.pack(side=tk.RIGHT, padx=12)

        # ── Cube parameters ───────────────────────────────────────────────
        cube_f = ttk.LabelFrame(main, text="📦  Cube", padding=8)
        cube_f.pack(fill=tk.X, pady=(0, 8))

        self.cube_size = self._labeled_entry(cube_f, "Size (cm):", "2", width=6, row=0, col=0)
        ttk.Label(cube_f, text="(cube is assumed to be a perfect cube — one value for all sides)",
                  font=('Arial', 8), foreground='gray').grid(row=0, column=3, padx=8, sticky='w')

        # ── Pick position ─────────────────────────────────────────────────
        pick_f = ttk.LabelFrame(main, text="📍  Pick Position  (floor where cube sits)", padding=8)
        pick_f.pack(fill=tk.X, pady=(0, 8))

        self.pick_x = self._labeled_entry(pick_f, "X:", "-10", row=0, col=0)
        self.pick_y = self._labeled_entry(pick_f, "Y:",  "10", row=0, col=2)
        self.pick_z = self._labeled_entry(pick_f, "Z (floor):", "0", row=0, col=4)
        ttk.Label(pick_f, text="cm", foreground='gray').grid(row=0, column=7, sticky='w')
        ttk.Button(pick_f, text="← Use current",
                   command=lambda: self._copy_current_pos(
                       self.pick_x, self.pick_y, self.pick_z)).grid(row=0, column=8, padx=6)

        # ── Place position ────────────────────────────────────────────────
        place_f = ttk.LabelFrame(main, text="📍  Place Position  (floor where cube is dropped)", padding=8)
        place_f.pack(fill=tk.X, pady=(0, 8))

        self.place_x = self._labeled_entry(place_f, "X:", "10", row=0, col=0)
        self.place_y = self._labeled_entry(place_f, "Y:", "10", row=0, col=2)
        self.place_z = self._labeled_entry(place_f, "Z (floor):", "0", row=0, col=4)
        ttk.Label(place_f, text="cm", foreground='gray').grid(row=0, column=7, sticky='w')
        ttk.Button(place_f, text="← Use current",
                   command=lambda: self._copy_current_pos(
                       self.place_x, self.place_y, self.place_z)).grid(row=0, column=8, padx=6)

        # ── Motion settings ───────────────────────────────────────────────
        motion_f = ttk.LabelFrame(main, text="⚙️  Motion Settings", padding=8)
        motion_f.pack(fill=tk.X, pady=(0, 8))

        row1 = ttk.Frame(motion_f)
        row1.pack(fill=tk.X, pady=2)
        self.approach_h = self._labeled_entry(row1, "Approach height (cm):", "5", width=6,
                                              row=0, col=0, inline=True)
        self.retract_h  = self._labeled_entry(row1, "  Retract height (cm):", "5", width=6,
                                              row=0, col=2, inline=True)

        row2 = ttk.Frame(motion_f)
        row2.pack(fill=tk.X, pady=2)
        self.n_approach = self._labeled_entry(row2, "Steps per segment:", "25", width=5,
                                              row=0, col=0, inline=True)
        ttk.Label(row2, text="   (joint-space interpolation steps between each key pose)",
                  font=('Arial', 8), foreground='gray').grid(row=0, column=3, padx=8, sticky='w')

        row3 = ttk.Frame(motion_f)
        row3.pack(fill=tk.X, pady=2)
        self.gripper_open_v  = self._labeled_entry(row3, "Gripper open (°):",  "30", width=5,
                                                   row=0, col=0, inline=True)
        self.gripper_close_v = self._labeled_entry(row3, "  Gripper close (°):", "80", width=5,
                                                   row=0, col=2, inline=True)

        row5 = ttk.Frame(motion_f)
        row5.pack(fill=tk.X, pady=2)
        ttk.Label(row5, text="Transit dt (s):").pack(side=tk.LEFT, padx=4)
        self.dt_var = tk.DoubleVar(value=0.07)
        self.dt_slider = ttk.Scale(row5, from_=0.01, to=0.2, orient=tk.HORIZONTAL,
                                   variable=self.dt_var, length=200,
                                   command=lambda v: self.dt_label.config(
                                       text=f"{float(v):.3f} s"))
        self.dt_slider.pack(side=tk.LEFT, padx=4)
        self.dt_label = ttk.Label(row5, text="0.070 s", width=8)
        self.dt_label.pack(side=tk.LEFT)
        ttk.Label(row5, text="(phases 1 & 4: get-to-pos / transfer)",
                  font=('Arial', 8), foreground='gray').pack(side=tk.LEFT, padx=4)

        row6 = ttk.Frame(motion_f)
        row6.pack(fill=tk.X, pady=2)
        ttk.Label(row6, text="Vertical dt (s):").pack(side=tk.LEFT, padx=4)
        self.vdt_var = tk.DoubleVar(value=0.14)
        self.vdt_slider = ttk.Scale(row6, from_=0.01, to=0.4, orient=tk.HORIZONTAL,
                                    variable=self.vdt_var, length=200,
                                    command=lambda v: self.vdt_label.config(
                                        text=f"{float(v):.3f} s"))
        self.vdt_slider.pack(side=tk.LEFT, padx=4)
        self.vdt_label = ttk.Label(row6, text="0.140 s", width=8)
        self.vdt_label.pack(side=tk.LEFT)
        ttk.Label(row6, text="(phases 2,3,5,6: descend / retract / drop)",
                  font=('Arial', 8), foreground='gray').pack(side=tk.LEFT, padx=4)

        self.return_home_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(motion_f, text="Return to home after drop",
                        variable=self.return_home_var).pack(anchor='w', padx=4, pady=2)

        # ── Action buttons ────────────────────────────────────────────────
        btn_f = ttk.Frame(main)
        btn_f.pack(fill=tk.X, pady=(4, 6))

        self.plan_btn = ttk.Button(btn_f, text="🗺  Plan Path",
                                   command=self._plan, width=18)
        self.plan_btn.pack(side=tk.LEFT, padx=4)

        self.exec_btn = ttk.Button(btn_f, text="▶  Execute",
                                   command=self._execute, width=18, state='disabled')
        self.exec_btn.pack(side=tk.LEFT, padx=4)

        self.stop_btn = ttk.Button(btn_f, text="⏹  Stop",
                                   command=self._stop, width=10, state='disabled')
        self.stop_btn.pack(side=tk.LEFT, padx=4)

        self.home_btn = ttk.Button(btn_f, text="🏠 Home",
                                   command=self._go_home, width=10)
        self.home_btn.pack(side=tk.RIGHT, padx=4)

        # ── Log ───────────────────────────────────────────────────────────
        log_f = ttk.LabelFrame(main, text="Log", padding=4)
        log_f.pack(fill=tk.BOTH, expand=True)

        self.log = scrolledtext.ScrolledText(log_f, height=10, font=('Consolas', 9),
                                             state='disabled', wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)

        # ── Status bar ────────────────────────────────────────────────────
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(main, textvariable=self.status_var, font=('Arial', 9),
                  foreground='gray', anchor='w').pack(fill=tk.X, pady=(2, 0))

    # ─────────────────────────────────────────── helpers ──────────────────
    def _labeled_entry(self, parent, text, default, width=8, row=0, col=0, inline=False):
        """Create a label+entry pair inside a grid/pack container. Returns the Entry widget."""
        if inline:
            ttk.Label(parent, text=text).grid(row=row, column=col, sticky='e', padx=(4, 2))
            e = ttk.Entry(parent, width=width)
            e.insert(0, default)
            e.grid(row=row, column=col + 1, padx=(0, 4))
        else:
            ttk.Label(parent, text=text).grid(row=row, column=col, sticky='e', padx=(4, 2))
            e = ttk.Entry(parent, width=width)
            e.insert(0, default)
            e.grid(row=row, column=col + 1, padx=(0, 6))
        return e

    def _log(self, msg):
        self.log.configure(state='normal')
        self.log.insert(tk.END, msg + "\n")
        self.log.see(tk.END)
        self.log.configure(state='disabled')

    def _status(self, msg, color='gray'):
        self.root.after(0, lambda: self.status_var.set(msg))

    def _get_float(self, entry, name):
        try:
            return float(entry.get())
        except ValueError:
            raise ValueError(f"'{name}' must be a number")

    def _get_int(self, entry, name):
        try:
            return int(entry.get())
        except ValueError:
            raise ValueError(f"'{name}' must be an integer")

    def _copy_current_pos(self, ex, ey, ez):
        if not self.robot:
            messagebox.showwarning("Not connected", "Connect to the robot first.")
            return
        try:
            x, y, z = self.robot.get_end_effector_position()
            for widget, val in [(ex, x), (ey, y), (ez, z)]:
                widget.delete(0, tk.END)
                widget.insert(0, f"{val:.1f}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    # ──────────────────────────────────────── connection ──────────────────
    def _connect_robot(self):
        def do_connect():
            self.root.after(0, lambda: self.conn_label.config(
                text="🟡 Connecting…", foreground='orange'))
            self._log(f"Connecting to {self.port_var.get()}…")
            try:
                if self.robot:
                    self.robot.disconnect()
                self.robot = RobotArmController(port=self.port_var.get())
                if self.robot.connect():
                    angles = self.robot.get_current_angles()
                    x, y, z = self.robot.get_end_effector_position()
                    self.root.after(0, lambda: [
                        self.conn_label.config(text="🟢 Connected", foreground='green'),
                        self.pos_label.config(
                            text=f"Pos: ({x:.1f}, {y:.1f}, {z:.1f})  "
                                 f"b={angles['b']}° s={angles['s']}° "
                                 f"e={angles['e']}° w={angles['w']}°"),
                    ])
                    self._log(f"✓ Connected. Angles: b={angles['b']}° s={angles['s']}° "
                              f"e={angles['e']}° w={angles['w']}°")
                    self._log(f"  End-effector: ({x:.1f}, {y:.1f}, {z:.1f}) cm")
                else:
                    self.root.after(0, lambda: self.conn_label.config(
                        text="🔴 Failed", foreground='red'))
                    self._log("✗ Connection failed")
                    self.robot = None
            except Exception as e:
                self.root.after(0, lambda: self.conn_label.config(
                    text="🔴 Error", foreground='red'))
                self._log(f"✗ Error: {e}")
                self.robot = None

        threading.Thread(target=do_connect, daemon=True).start()

    # ───────────────────────────────────────────── plan ───────────────────
    def _plan(self):
        """Validate inputs and show IK preview for the 5 key positions."""
        try:
            cube_size    = self._get_float(self.cube_size,    "Cube size")
            pick_x       = self._get_float(self.pick_x,      "Pick X")
            pick_y       = self._get_float(self.pick_y,      "Pick Y")
            pick_floor_z = self._get_float(self.pick_z,      "Pick Z (floor)")
            place_x      = self._get_float(self.place_x,     "Place X")
            place_y      = self._get_float(self.place_y,     "Place Y")
            place_floor_z= self._get_float(self.place_z,     "Place Z (floor)")
            approach_h   = self._get_float(self.approach_h,  "Approach height")
            retract_h    = self._get_float(self.retract_h,   "Retract height")
            steps        = self._get_int(self.n_approach,    "Steps per segment")
            gripper_open  = self._get_int(self.gripper_open_v,  "Gripper open")
            gripper_close = self._get_int(self.gripper_close_v, "Gripper close")
            dt          = self.dt_var.get()
            vertical_dt = self.vdt_var.get()
        except ValueError as e:
            messagebox.showerror("Invalid input", str(e))
            return

        if not self.robot:
            messagebox.showwarning("Not connected", "Connect to the robot first.")
            return

        # Grasp height = centre of the cube
        pick_pos  = (pick_x,  pick_y,  pick_floor_z  + cube_size / 2)
        place_pos = (place_x, place_y, place_floor_z + cube_size / 2)

        self._log("\n" + "="*60)
        self._log("IK Preview (joint-space interpolation)")
        self._log(f"  Cube size: {cube_size} cm  →  grasp Z = {pick_pos[2]:.1f} cm")
        self._log(f"  Pick:  ({pick_pos[0]}, {pick_pos[1]}, {pick_pos[2]:.1f})")
        self._log(f"  Place: ({place_pos[0]}, {place_pos[1]}, {place_pos[2]:.1f})")
        self._log(f"  Steps/segment: {steps}   transit dt: {dt:.3f} s   vertical dt: {vertical_dt:.3f} s")

        # Solve IK for the 5 key positions
        from inverse_kinematics_servo import calculate_ik
        px, py, pz = pick_pos
        lx, ly, lz = place_pos
        cur = self.robot.get_current_angles()
        guess = {k: cur[k] for k in ('b', 's', 'e', 'w')}
        ok_all = True
        for label, pos in [
            ('above_pick',  (px, py, pz + approach_h)),
            ('at_pick',     (px, py, pz)),
            ('above_place', (lx, ly, lz + retract_h)),
            ('at_place',    (lx, ly, lz)),
        ]:
            angles, ok, err = calculate_ik(*pos, guess)
            st = '✓' if ok else '✗'
            if not ok:
                ok_all = False
            self._log(f"  {st} {label:14s}: b={angles['b']:3d}° s={angles['s']:3d}° "
                      f"e={angles['e']:3d}° w={angles['w']:3d}°  err={err:.2f} cm")
            guess = angles

        if not ok_all:
            self._log("  ⚠️  Some positions have high IK error — check reachability")

        self._log("\n✓ Preview done — all positions computed. Click Execute to run.")

        # Store params for _execute
        self.planned_params = dict(
            pick_pos=pick_pos, place_pos=place_pos,
            approach_height=approach_h, retract_height=retract_h,
            gripper_open=gripper_open, gripper_close=gripper_close,
            steps_per_segment=steps, dt=dt, vertical_dt=vertical_dt,
        )
        self.exec_btn.config(state='normal')
        self._status("Preview done — ready to execute", 'blue')

    # ─────────────────────────────────────────── execute ──────────────────
    def _execute(self):
        if not self.planned_params:
            messagebox.showwarning("No plan", "Run 'Plan Path' first.")
            return
        if not self.robot:
            messagebox.showwarning("Not connected", "Connect to the robot first.")
            return
        if self.executing:
            return

        self.executing = True
        self._stop_requested = False
        self.exec_btn.config(state='disabled')
        self.plan_btn.config(state='disabled')
        self.stop_btn.config(state='normal')

        params = self.planned_params

        def run():
            self._log("\n" + "="*60)
            self._log("EXECUTING PICK AND PLACE")
            self._log("="*60)
            self._status("Executing…", 'blue')
            try:
                success = self.robot.execute_pick_and_place(
                    pick_pos=params['pick_pos'],
                    place_pos=params['place_pos'],
                    approach_height=params['approach_height'],
                    retract_height=params['retract_height'],
                    gripper_open=params['gripper_open'],
                    gripper_close=params['gripper_close'],
                    steps_per_segment=params['steps_per_segment'],
                    dt=params['dt'],
                    vertical_dt=params['vertical_dt'],
                    display_progress=True
                )

                if success:
                    final = self.robot.get_end_effector_position()
                    self._log(f"\n✓ Completed! Final pos: ({final[0]:.1f}, {final[1]:.1f}, {final[2]:.1f})")
                    self._status("Completed successfully", 'green')

                    if self.return_home_var.get() and not self._stop_requested:
                        self._log("🏠 Returning to home…")
                        self.robot.home_position(smooth=True, display_progress=False)
                        home = self.robot.get_end_effector_position()
                        self._log(f"✓ Home reached: ({home[0]:.1f}, {home[1]:.1f}, {home[2]:.1f})")
                        self._status("Home position reached", 'green')
                else:
                    self._log("✗ Execution reported failure")
                    self._status("Execution failed", 'red')

            except Exception as e:
                self._log(f"✗ Error: {e}")
                self._status(f"Error: {e}", 'red')
            finally:
                self.executing = False
                self.root.after(0, lambda: [
                    self.exec_btn.config(state='normal'),
                    self.plan_btn.config(state='normal'),
                    self.stop_btn.config(state='disabled'),
                ])

        threading.Thread(target=run, daemon=True).start()

    def _stop(self):
        self._stop_requested = True
        self._log("⏹ Stop requested — will halt after current waypoint")
        self._status("Stopping…", 'orange')

    def _go_home(self):
        if not self.robot:
            messagebox.showwarning("Not connected", "Connect to the robot first.")
            return

        def do_home():
            self._log("🏠 Moving to home…")
            self._status("Moving to home…", 'blue')
            try:
                self.robot.home_position(smooth=True, display_progress=False)
                home = self.robot.get_end_effector_position()
                self._log(f"✓ Home reached: ({home[0]:.1f}, {home[1]:.1f}, {home[2]:.1f})")
                self._status("Home position reached", 'green')
            except Exception as e:
                self._log(f"✗ Error: {e}")

        threading.Thread(target=do_home, daemon=True).start()

    def on_closing(self):
        if self.robot:
            self.robot.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = CubePickupGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
