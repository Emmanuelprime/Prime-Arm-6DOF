#!/usr/bin/env python3
"""
Joystick Controller for Prime-Arm
Receives UDP JSON packets from the drone-controller ESP32 joystick and maps
stick axes to robot joint angles via smooth joint-space control.

Default axis → joint mapping:
  Right X (rx)  →  Base     (b)
  Right Y (ry)  →  Shoulder (s)
  Left  Y (ly)  →  Elbow    (e)
  Left  X (lx)  →  Wrist    (w)

Gripper is toggled via the GUI button or G key.
Home is triggered via the GUI button or H key.
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import socket
import threading
import json
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from raspberry_pi_controller import RobotArmController, DEFAULT_PORT

# ───────────────────────────────────────────────────────────── constants ──────
UDP_PORT    = 4210
UPDATE_HZ   = 50
DT          = 1.0 / UPDATE_HZ
JOY_TIMEOUT = 0.5        # seconds — treat axes as zero if no packet this long

JOINTS      = ('b', 's', 'e', 'w', 't')
JOINT_NAMES = {'b': 'Base', 's': 'Shoulder', 'e': 'Elbow',
               'w': 'Wrist', 't': 'Twist'}
LIMITS      = {'b': (0, 180), 's': (0, 180), 'e': (0, 180),
               'w': (0, 180), 't': (0, 180)}
HOME        = {'b': 90, 's': 130, 'e': 180, 'w': 180, 't': 180, 'g': 80}
G_OPEN      = 60
G_CLOSE     = 80

AXES        = ('rx', 'ry', 'lx', 'ly')
AXIS_LABELS = {'rx': 'Right X', 'ry': 'Right Y',
               'lx': 'Left X',  'ly': 'Left Y'}
# axis → (joint, default_sign)
DEFAULT_MAP = {'rx': ('b', +1), 'ry': ('s', +1),
               'ly': ('e', +1), 'lx': ('w', +1)}


# ─────────────────────────────────────────────────────────────────── GUI ──────
class JoystickController:

    def __init__(self, root):
        self.root = root
        self.root.title("Prime-Arm Joystick Controller")
        self.root.geometry("820x620")
        self.root.resizable(True, True)

        # Robot state  (mirrors gui_controller)
        self.robot           = None
        self.connected       = False
        self.sending_command = False

        # Joint targets
        self.target_angles = {j: HOME[j] for j in JOINTS}
        self.gripper       = G_CLOSE

        # Joystick state
        self._joy_lock   = threading.Lock()
        self._joy        = {a: 0.0 for a in AXES}
        self._last_joy_t = 0.0
        self._packets    = 0

        # Axis → joint mapping (GUI-configurable)
        self._map_joint = {a: tk.StringVar(value=DEFAULT_MAP[a][0]) for a in AXES}
        self._map_sign  = {a: tk.IntVar(value=DEFAULT_MAP[a][1])    for a in AXES}

        # Speed (°/s at full deflection)
        self.speed_var = tk.DoubleVar(value=60.0)

        # Control loop flag
        self._control_running = False
        self._homing = False

        self.create_widgets()
        self._start_udp_listener()
        self._start_control_loop()

        self.root.bind('<g>', lambda e: self.toggle_gripper())
        self.root.bind('<G>', lambda e: self.toggle_gripper())
        self.root.bind('<h>', lambda e: self.go_home())
        self.root.bind('<H>', lambda e: self.go_home())

    # ══════════════════════════════════════════════════════════ create_widgets ═
    def create_widgets(self):
        """Create GUI widgets — mirrors gui_controller.create_widgets."""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)

        # ── Title ─────────────────────────────────────────────────────────────
        ttk.Label(main_frame, text="Prime-Arm Joystick Controller",
                  font=('Arial', 16, 'bold')).grid(
            row=0, column=0, columnspan=2, pady=(0, 10))

        # ── Connection (mirrors gui_controller connection frame) ───────────────
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.grid(row=1, column=0, columnspan=2,
                        sticky=(tk.W, tk.E), pady=(0, 10))
        conn_frame.columnconfigure(1, weight=1)

        ttk.Label(conn_frame, text="Port:").grid(
            row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.port_entry = ttk.Entry(conn_frame, width=20)
        self.port_entry.insert(0, DEFAULT_PORT)
        self.port_entry.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(0, 10))

        self.connect_btn = ttk.Button(conn_frame, text="Connect",
                                      command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2)

        self.status_label = ttk.Label(conn_frame, text="● Disconnected",
                                      foreground="red")
        self.status_label.grid(row=0, column=3, padx=(10, 0))

        self.udp_label = ttk.Label(conn_frame,
                                   text=f"  |  UDP :{UDP_PORT} …",
                                   foreground="gray")
        self.udp_label.grid(row=0, column=4, padx=(10, 0))

        self.pkt_label = ttk.Label(conn_frame, text="pkts: 0",
                                   font=('Consolas', 8), foreground="gray")
        self.pkt_label.grid(row=0, column=5, padx=(10, 0))

        # ── Left column ───────────────────────────────────────────────────────
        left = ttk.Frame(main_frame)
        left.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        left.columnconfigure(0, weight=1)

        # Live axes
        axes_frame = ttk.LabelFrame(left, text="Live Axes", padding="10")
        axes_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        axes_frame.columnconfigure(1, weight=1)

        self.axis_bars = {}
        self.axis_vals = {}
        for r, ax in enumerate(AXES):
            ttk.Label(axes_frame, text=AXIS_LABELS[ax] + ":",
                      width=10, anchor='e').grid(
                row=r, column=0, sticky='e', padx=(0, 4))
            bar = ttk.Progressbar(axes_frame, orient='horizontal',
                                  length=160, maximum=200, value=100)
            bar.grid(row=r, column=1, sticky='ew', pady=1)
            val = ttk.Label(axes_frame, text=" 0.000",
                            font=('Consolas', 9), width=7)
            val.grid(row=r, column=2, sticky='w', padx=4)
            self.axis_bars[ax] = bar
            self.axis_vals[ax] = val

        # Joint angles (mirrors gui_controller servo controls)
        joints_frame = ttk.LabelFrame(left, text="Joint Angles", padding="10")
        joints_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        joints_frame.columnconfigure(1, weight=1)

        self.angle_bars   = {}
        self.angle_labels = {}
        for r, j in enumerate(JOINTS):
            ttk.Label(joints_frame, text=f"{JOINT_NAMES[j]}:",
                      font=('Arial', 10, 'bold'), width=10).grid(
                row=r, column=0, sticky=tk.W, pady=5)
            bar = ttk.Progressbar(joints_frame, orient='horizontal',
                                  length=160, maximum=180,
                                  value=self.target_angles[j])
            bar.grid(row=r, column=1, sticky='ew', padx=(10, 10))
            lbl = ttk.Label(joints_frame,
                            text=f"{self.target_angles[j]:3d}°",
                            font=('Arial', 10), width=6)
            lbl.grid(row=r, column=2, sticky=tk.E)
            self.angle_bars[j]   = bar
            self.angle_labels[j] = lbl

        # Control buttons (mirrors gui_controller button_frame)
        ctrl_frame = ttk.LabelFrame(left, text="Control", padding="10")
        ctrl_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 10))

        self.home_btn = ttk.Button(ctrl_frame, text="Home Position (H)",
                                   command=self.go_home, state='disabled')
        self.home_btn.grid(row=0, column=0, padx=5)

        self.grip_btn = ttk.Button(ctrl_frame, text="Open Gripper (G)",
                                   command=self.toggle_gripper, state='disabled')
        self.grip_btn.grid(row=0, column=1, padx=5)

        self.grip_lbl = ttk.Label(ctrl_frame, text="CLOSED",
                                  foreground='red', font=('Arial', 9, 'bold'))
        self.grip_lbl.grid(row=0, column=2, padx=(8, 0))

        # ── Right column ──────────────────────────────────────────────────────
        right = ttk.Frame(main_frame)
        right.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)

        # Speed slider
        spd_frame = ttk.LabelFrame(right, text="Speed  (°/s at full stick)",
                                   padding="10")
        spd_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        ttk.Scale(spd_frame, variable=self.speed_var,
                  from_=5, to=180, orient='horizontal',
                  command=lambda v: self._update_speed_label()).pack(
            fill='x', padx=4)
        self.spd_lbl = ttk.Label(spd_frame, font=('Consolas', 9))
        self.spd_lbl.pack()
        self._update_speed_label()

        # End-effector position (mirrors gui_controller pos_frame)
        pos_frame = ttk.LabelFrame(right, text="End Effector Position (cm)",
                                   padding="10")
        pos_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        self.x_label = ttk.Label(pos_frame, text="X: ---",
                                 font=('Arial', 11, 'bold'), width=12)
        self.x_label.grid(row=0, column=0, padx=6)
        self.y_label = ttk.Label(pos_frame, text="Y: ---",
                                 font=('Arial', 11, 'bold'), width=12)
        self.y_label.grid(row=0, column=1, padx=6)
        self.z_label = ttk.Label(pos_frame, text="Z: ---",
                                 font=('Arial', 11, 'bold'), width=12)
        self.z_label.grid(row=0, column=2, padx=6)

        # Axis → joint mapping
        map_frame = ttk.LabelFrame(right, text="Axis → Joint Mapping",
                                   padding="10")
        map_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        map_frame.columnconfigure(1, weight=1)

        joint_opts = list(JOINTS) + ['—']
        for r, ax in enumerate(AXES):
            ttk.Label(map_frame, text=AXIS_LABELS[ax] + ":",
                      width=10, anchor='e').grid(
                row=r, column=0, sticky='e', padx=(0, 4), pady=2)
            ttk.Combobox(map_frame, textvariable=self._map_joint[ax],
                         values=joint_opts, width=8,
                         state='readonly').grid(
                row=r, column=1, sticky='w', padx=4)
            sf = ttk.Frame(map_frame)
            sf.grid(row=r, column=2, sticky='w')
            ttk.Radiobutton(sf, text="+",
                            variable=self._map_sign[ax], value=+1).pack(side=tk.LEFT)
            ttk.Radiobutton(sf, text="−",
                            variable=self._map_sign[ax], value=-1).pack(side=tk.LEFT)

        # Log
        log_frame = ttk.LabelFrame(right, text="Log", padding=4)
        log_frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        right.rowconfigure(3, weight=1)
        self.log_box = scrolledtext.ScrolledText(log_frame, height=10,
                                                 font=('Consolas', 8),
                                                 state='disabled', wrap=tk.WORD)
        self.log_box.grid(row=0, column=0, sticky='nsew')

        # ── Status bar (mirrors gui_controller) ───────────────────────────────
        status_bar = ttk.Frame(main_frame, relief=tk.SUNKEN)
        status_bar.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E))
        self.info_label = ttk.Label(status_bar,
                                    text="Ready. Please connect to Arduino.",
                                    font=('Arial', 9))
        self.info_label.pack(side=tk.LEFT, padx=5, pady=2)

    # ══════════════════════════════════════════════════════════════ connection ═
    def toggle_connection(self):
        """Mirrors gui_controller.toggle_connection."""
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        """Mirrors gui_controller.connect — synchronous, shows messagebox on error."""
        port = self.port_entry.get()
        try:
            self.update_info("Connecting...")
            self.robot = RobotArmController(port=port, baudrate=115200,
                                            step_delay=0.015)
            if self.robot.connect():
                self.connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="● Connected", foreground="green")
                self.home_btn.config(state='normal')
                self.grip_btn.config(state='normal')
                self.port_entry.config(state='disabled')

                # Sync local targets with actual robot angles (like refresh_angles)
                self.refresh_angles()

                self.update_info("Connected successfully!")
                self._log(f"Robot connected on {port}.")
            else:
                messagebox.showerror("Connection Error",
                                     "Failed to connect to Arduino")
                self.update_info("Connection failed")
                self.robot = None
        except Exception as e:
            messagebox.showerror("Error", f"Connection error: {e}")
            self.update_info(f"Error: {e}")
            self.robot = None

    def disconnect(self):
        """Mirrors gui_controller.disconnect."""
        if self.robot:
            self.robot.disconnect()
            self.robot = None
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="● Disconnected", foreground="red")
        self.home_btn.config(state='disabled')
        self.grip_btn.config(state='disabled')
        self.port_entry.config(state='normal')
        self.update_info("Disconnected")
        self._log("Robot disconnected.")

    def refresh_angles(self):
        """Read current angles from the robot and sync target_angles + display.
        Mirrors gui_controller.refresh_angles."""
        if not self.connected:
            return
        try:
            actual = self.robot.get_current_angles()
            for j in JOINTS:
                self.target_angles[j] = actual[j]
            self.gripper = actual.get('g', G_CLOSE)
            self._update_joint_display(self.target_angles)
            self.update_info("Angles refreshed")
        except Exception as e:
            self.update_info(f"Error refreshing angles: {e}")

    # ═══════════════════════════════════════════════════════════ UDP listener ═
    def _start_udp_listener(self):
        threading.Thread(target=self._udp_loop, daemon=True).start()

    def _udp_loop(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', UDP_PORT))
            sock.settimeout(1.0)
            self.root.after(0, lambda: self.udp_label.config(
                text=f"  |  UDP :{UDP_PORT} listening",
                foreground='green'))
            self._log(f"UDP listening on port {UDP_PORT}.")
        except Exception as e:
            self.root.after(0, lambda: self.udp_label.config(
                text=f"  |  UDP error: {e}", foreground='red'))
            self._log(f"UDP bind error: {e}")
            return

        while True:
            try:
                data, _ = sock.recvfrom(256)
                pkt = json.loads(data.decode())
                with self._joy_lock:
                    for ax in AXES:
                        self._joy[ax] = float(pkt.get(ax, 0.0))
                    self._last_joy_t = time.monotonic()
                    self._packets += 1
            except socket.timeout:
                continue
            except (json.JSONDecodeError, ValueError):
                continue
            except Exception:
                break
        sock.close()

    # ════════════════════════════════════════════════════════ control loop ══════
    def _start_control_loop(self):
        self._control_running = True
        threading.Thread(target=self._control_loop, daemon=True).start()

    def _control_loop(self):
        while self._control_running:
            t0 = time.monotonic()
            self._tick()
            elapsed = time.monotonic() - t0
            remaining = DT - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _tick(self):
        """One control cycle: read joystick, compute deltas, send command.
        Uses sending_command guard — same pattern as gui_controller."""
        now = time.monotonic()
        with self._joy_lock:
            stale = (now - self._last_joy_t) > JOY_TIMEOUT
            axes  = {a: 0.0 for a in AXES} if stale else dict(self._joy)
            pkts  = self._packets

        # Always refresh axis display
        self.root.after(0, lambda a=dict(axes), p=pkts, s=stale:
                        self._update_axis_display(a, p, s))

        if not self.connected or self.sending_command:
            return

        speed      = self.speed_var.get()
        max_step   = speed * DT
        changed    = False
        new_angles = dict(self.target_angles)

        if self._homing:
            # Drive every joint toward HOME at current speed
            all_home = True
            for j in JOINTS:
                diff = HOME[j] - new_angles[j]
                if abs(diff) > 0.5:
                    step = max_step if diff > 0 else -max_step
                    if abs(diff) < abs(step):
                        step = diff
                    new_angles[j] += step
                    all_home = False
            if all_home:
                self._homing = False
                self.root.after(0, lambda: self.update_info("Home position reached"))
                self.root.after(0, lambda: self._log("Home reached."))
            changed = not all_home
        else:
            # Normal joystick rate control
            for ax in AXES:
                joint = self._map_joint[ax].get()
                if joint == '—' or joint not in LIMITS:
                    continue
                sign  = self._map_sign[ax].get()
                delta = axes[ax] * sign * speed * DT
                if abs(delta) < 0.01:
                    continue
                lo, hi = LIMITS[joint]
                new_angles[joint] = max(lo, min(hi, new_angles[joint] + delta))
                changed = True

        if not changed:
            return

        self.target_angles = new_angles
        cmd = {j: int(round(new_angles[j])) for j in JOINTS}
        cmd['g'] = self.gripper

        # Send in background thread with sending_command guard
        # (mirrors gui_controller.send_servo_command)
        self.sending_command = True

        def send(c=cmd, a=dict(new_angles)):
            try:
                self.robot._stream_command(c)
            except Exception as e:
                self.root.after(0, lambda: self.update_info(f"Error: {e}"))
            finally:
                self.sending_command = False
            self.root.after(0, lambda: self._update_joint_display(a))
            # Compute FK from target angles (no serial needed) and update XYZ
            try:
                x, y, z = self.robot.calculate_fk(a)
                self.root.after(0, lambda: self._update_position_display(x, y, z))
            except Exception:
                pass

        threading.Thread(target=send, daemon=True).start()

    # ════════════════════════════════════════════════════ gripper / home ═══════
    def toggle_gripper(self):
        """Mirrors gui_controller button handler style."""
        if not self.connected or self.sending_command:
            return
        self.gripper = G_OPEN if self.gripper == G_CLOSE else G_CLOSE
        label  = "OPEN"  if self.gripper == G_OPEN  else "CLOSED"
        colour = 'green' if self.gripper == G_OPEN  else 'red'
        btn_t  = ("Close Gripper (G)" if self.gripper == G_OPEN
                  else "Open Gripper (G)")
        self.grip_lbl.config(text=label, foreground=colour)
        self.grip_btn.config(text=btn_t)

        cmd = {j: int(round(self.target_angles[j])) for j in JOINTS}
        cmd['g'] = self.gripper

        self.sending_command = True

        def send():
            try:
                self.robot._stream_command(cmd)
                self.update_info(f"Gripper {label.lower()}")
            except Exception as e:
                self.update_info(f"Error: {e}")
            finally:
                self.sending_command = False

        threading.Thread(target=send, daemon=True).start()

    def go_home(self):
        """Activate homing — _tick drives every joint to HOME at current speed."""
        if not self.connected:
            return
        self._homing = True
        self.update_info("Going home...")
        self._log("Going home.")

    # ═════════════════════════════════════════════════════════ display helpers ═
    def _update_position_display(self, x, y, z):
        self.x_label.config(text=f"X: {x:6.2f}")
        self.y_label.config(text=f"Y: {y:6.2f}")
        self.z_label.config(text=f"Z: {z:6.2f}")

    def _update_joint_display(self, angles):
        for j in JOINTS:
            a = int(round(angles.get(j, self.target_angles.get(j, 0))))
            self.angle_bars[j]['value'] = a
            self.angle_labels[j].config(text=f"{a:3d}°")

    def _update_axis_display(self, axes, pkts, stale):
        for ax in AXES:
            v = axes.get(ax, 0.0)
            # −1..+1  →  0..200  (100 = centre)
            self.axis_bars[ax]['value'] = int((v + 1.0) * 100)
            self.axis_vals[ax].config(text=f"{v:+.3f}")
        self.pkt_label.config(text=f"pkts: {pkts}")
        if stale and pkts > 0:
            self.udp_label.config(text="  |  Joystick silent…",
                                  foreground='orange')
        elif pkts > 0:
            self.udp_label.config(text=f"  |  UDP :{UDP_PORT} receiving",
                                  foreground='green')

    def _update_speed_label(self):
        self.spd_lbl.config(text=f"{self.speed_var.get():.0f} °/s")

    def update_info(self, message):
        """Update status bar — mirrors gui_controller.update_info."""
        self.root.after(0, lambda: self.info_label.config(text=message))

    def _log(self, msg):
        def _do():
            self.log_box.configure(state='normal')
            self.log_box.insert(tk.END, msg + "\n")
            self.log_box.see(tk.END)
            self.log_box.configure(state='disabled')
        self.root.after(0, _do)

    # ══════════════════════════════════════════════════════════════ on_closing ═
    def on_closing(self):
        """Mirrors gui_controller.on_closing."""
        self._control_running = False
        if self.connected:
            self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app  = JoystickController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
