#!/usr/bin/env python3
"""
Joystick Controller for Prime-Arm
==================================
Receives UDP JSON packets from the drone-controller joystick firmware and maps
stick axes to robot joint angles via smooth joint-space control.

Default axis → joint mapping:
  Right X (rx)  →  Base     (b)
  Right Y (ry)  →  Shoulder (s)
  Left  Y (ly)  →  Elbow    (e)
  Left  X (lx)  →  Wrist    (w)

Twist is controlled by a GUI slider or left unmapped.
Gripper is toggled via a GUI button or the G key.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import socket
import threading
import json
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from raspberry_pi_controller import RobotArmController, DEFAULT_PORT

# ─────────────────────────────────────────────── constants ────────────────────
UDP_PORT    = 4210
UPDATE_HZ   = 50
DT          = 1.0 / UPDATE_HZ
JOY_TIMEOUT = 0.5   # seconds — treat axes as zero if silent for this long

JOINTS      = ('b', 's', 'e', 'w', 't')
JOINT_NAMES = {'b': 'Base', 's': 'Shoulder', 'e': 'Elbow',
               'w': 'Wrist', 't': 'Twist'}
LIMITS      = {'b': (0, 180), 's': (0, 180), 'e': (0, 180),
               'w': (0, 180), 't': (0, 180)}
HOME        = {'b': 90, 's': 130, 'e': 180, 'w': 180, 't': 180, 'g': 80}
G_OPEN      = 30
G_CLOSE     = 80

AXES        = ('rx', 'ry', 'lx', 'ly')
AXIS_LABELS = {'rx': 'Right X', 'ry': 'Right Y', 'lx': 'Left X', 'ly': 'Left Y'}

# axis → (joint, default_sign)  — sign can be flipped in the GUI
DEFAULT_MAP = {
    'rx': ('b', +1),
    'ry': ('s', +1),
    'ly': ('e', +1),
    'lx': ('w', +1),
}

# ─────────────────────────────────────────────────────────── GUI ──────────────
class JoystickController:

    def __init__(self, root):
        self.root  = root
        self.root.title("Prime-Arm — Joystick Controller")
        self.root.geometry("860x680")
        self.root.resizable(True, True)

        # Robot state
        self.robot   = None
        self.angles  = {**HOME}          # current target angles
        self.gripper = G_CLOSE           # start closed
        self._angle_lock = threading.Lock()

        # Joystick state
        self._joy_lock    = threading.Lock()
        self._joy         = {a: 0.0 for a in AXES}
        self._last_joy_t  = 0.0
        self._packets     = 0

        # Mapping: axis → [joint_var, sign_var]
        self._map_joint = {a: tk.StringVar(value=DEFAULT_MAP[a][0]) for a in AXES}
        self._map_sign  = {a: tk.IntVar(value=DEFAULT_MAP[a][1])    for a in AXES}

        # Speed (degrees / second at full deflection)
        self.speed_var = tk.DoubleVar(value=60.0)

        # Threads
        self._udp_running     = False
        self._control_running = False

        self._build_ui()
        self._start_udp_listener()
        self._start_control_loop()

        self.root.bind('<g>', lambda e: self._toggle_gripper())
        self.root.bind('<G>', lambda e: self._toggle_gripper())
        self.root.bind('<h>', lambda e: self._go_home())
        self.root.bind('<H>', lambda e: self._go_home())

    # ═══════════════════════════════════════════════════════════ build UI ══════
    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # ── Left column ──────────────────────────────────────────────────────
        left = ttk.Frame(self.root, padding=8)
        left.grid(row=0, column=0, sticky='nsew')
        left.columnconfigure(0, weight=1)

        # Robot connection
        conn_f = ttk.LabelFrame(left, text="Robot Connection", padding=6)
        conn_f.grid(row=0, column=0, sticky='ew', pady=(0, 8))

        ttk.Label(conn_f, text="Port:").pack(side=tk.LEFT)
        import sys as _sys
        self.port_var = tk.StringVar(
            value='COM4' if _sys.platform.startswith('win') else '/dev/ttyUSB0')
        ttk.Entry(conn_f, textvariable=self.port_var, width=14).pack(side=tk.LEFT, padx=4)
        self.conn_btn = ttk.Button(conn_f, text="Connect Robot",
                                   command=self._toggle_robot)
        self.conn_btn.pack(side=tk.LEFT, padx=4)
        self.conn_lbl = ttk.Label(conn_f, text="⚫ Not connected",
                                  foreground='red', font=('Arial', 9, 'bold'))
        self.conn_lbl.pack(side=tk.LEFT, padx=4)

        # UDP status
        udp_f = ttk.LabelFrame(left, text="UDP (Joystick)", padding=6)
        udp_f.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        self.udp_lbl = ttk.Label(udp_f, text=f"⚫ Listening on :{UDP_PORT}…",
                                 font=('Arial', 9))
        self.udp_lbl.pack(side=tk.LEFT)
        self.pkt_lbl = ttk.Label(udp_f, text="pkts: 0",
                                 font=('Consolas', 8), foreground='gray')
        self.pkt_lbl.pack(side=tk.RIGHT)

        # Axis bars
        axes_f = ttk.LabelFrame(left, text="Live Axes", padding=6)
        axes_f.grid(row=2, column=0, sticky='ew', pady=(0, 8))
        axes_f.columnconfigure(1, weight=1)

        self._axis_bars  = {}
        self._axis_vals  = {}
        for r, ax in enumerate(AXES):
            ttk.Label(axes_f, text=AXIS_LABELS[ax] + ":",
                      width=10, anchor='e').grid(row=r, column=0, sticky='e', padx=(0, 4))
            bar = ttk.Progressbar(axes_f, orient='horizontal',
                                  length=180, maximum=200, value=100)
            bar.grid(row=r, column=1, sticky='ew', pady=1)
            val = ttk.Label(axes_f, text=" 0.000", font=('Consolas', 9), width=7)
            val.grid(row=r, column=2, sticky='w', padx=4)
            self._axis_bars[ax] = bar
            self._axis_vals[ax] = val

        # Joint angles display
        jt_f = ttk.LabelFrame(left, text="Joint Angles", padding=6)
        jt_f.grid(row=3, column=0, sticky='ew', pady=(0, 8))
        jt_f.columnconfigure(1, weight=1)

        self._joint_bars = {}
        self._joint_lbls = {}
        for r, j in enumerate(JOINTS):
            ttk.Label(jt_f, text=f"{JOINT_NAMES[j]}:",
                      width=10, anchor='e').grid(row=r, column=0, sticky='e', padx=(0, 4))
            bar = ttk.Progressbar(jt_f, orient='horizontal',
                                  length=180, maximum=180, value=self.angles[j])
            bar.grid(row=r, column=1, sticky='ew', pady=1)
            lbl = ttk.Label(jt_f, text=f"{self.angles[j]:3d}°",
                            font=('Consolas', 9), width=5)
            lbl.grid(row=r, column=2, sticky='w', padx=4)
            self._joint_bars[j] = bar
            self._joint_lbls[j] = lbl

        # Gripper
        g_f = ttk.LabelFrame(left, text="Gripper  (G key)", padding=6)
        g_f.grid(row=4, column=0, sticky='ew', pady=(0, 8))
        self.grip_btn = ttk.Button(g_f, text="🤚 Open Gripper",
                                   command=self._toggle_gripper)
        self.grip_btn.pack(side=tk.LEFT, padx=4)
        self.grip_lbl = ttk.Label(g_f, text="CLOSED", foreground='red',
                                  font=('Arial', 9, 'bold'))
        self.grip_lbl.pack(side=tk.LEFT, padx=8)

        # Home button
        home_f = ttk.Frame(left)
        home_f.grid(row=5, column=0, sticky='ew', pady=(0, 4))
        ttk.Button(home_f, text="🏠 Home  (H key)",
                   command=self._go_home).pack(side=tk.LEFT, padx=2)

        # ── Right column ─────────────────────────────────────────────────────
        right = ttk.Frame(self.root, padding=8)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)
        right.rowconfigure(3, weight=1)

        # Speed
        spd_f = ttk.LabelFrame(right, text="Speed  (°/s at full stick)", padding=6)
        spd_f.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        ttk.Scale(spd_f, variable=self.speed_var,
                  from_=5, to=180, orient='horizontal').pack(fill='x', padx=4)
        self.spd_lbl = ttk.Label(spd_f, font=('Consolas', 9))
        self.spd_lbl.pack()
        self.speed_var.trace_add('write', lambda *_: self._update_speed_label())
        self._update_speed_label()

        # Axis mapping
        map_f = ttk.LabelFrame(right, text="Axis → Joint Mapping", padding=6)
        map_f.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        map_f.columnconfigure(1, weight=1)

        joint_opts = list(JOINTS) + ['—']
        for r, ax in enumerate(AXES):
            ttk.Label(map_f, text=AXIS_LABELS[ax] + ":",
                      width=10, anchor='e').grid(row=r, column=0, sticky='e', padx=(0, 4), pady=2)
            cb = ttk.Combobox(map_f, textvariable=self._map_joint[ax],
                              values=joint_opts, width=8, state='readonly')
            cb.grid(row=r, column=1, sticky='w', padx=4)
            sign_frame = ttk.Frame(map_f)
            sign_frame.grid(row=r, column=2, sticky='w')
            ttk.Radiobutton(sign_frame, text="+", variable=self._map_sign[ax],
                            value=+1).pack(side=tk.LEFT)
            ttk.Radiobutton(sign_frame, text="−", variable=self._map_sign[ax],
                            value=-1).pack(side=tk.LEFT)

        # Tips
        tips_f = ttk.LabelFrame(right, text="Keyboard Shortcuts", padding=6)
        tips_f.grid(row=2, column=0, sticky='ew', pady=(0, 8))
        tips = [("G", "Toggle gripper"), ("H", "Go home")]
        for i, (key, desc) in enumerate(tips):
            ttk.Label(tips_f, text=key, font=('Consolas', 9, 'bold'),
                      width=4).grid(row=i, column=0, sticky='e')
            ttk.Label(tips_f, text=desc, foreground='gray').grid(
                row=i, column=1, sticky='w', padx=6)

        # Log
        log_f = ttk.LabelFrame(right, text="Log", padding=4)
        log_f.grid(row=3, column=0, sticky='nsew')
        log_f.columnconfigure(0, weight=1)
        log_f.rowconfigure(0, weight=1)
        self.log = scrolledtext.ScrolledText(log_f, height=12, font=('Consolas', 8),
                                              state='disabled', wrap=tk.WORD)
        self.log.grid(row=0, column=0, sticky='nsew')

    # ═══════════════════════════════════════════════ robot connection ══════════
    def _toggle_robot(self):
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None
            self.conn_btn.config(text="Connect Robot")
            self.conn_lbl.config(text="⚫ Not connected", foreground='red')
            self._log("Robot disconnected.")
            return

        port = self.port_var.get().strip()
        self.conn_lbl.config(text="🟡 Connecting…", foreground='orange')
        self.conn_btn.config(state='disabled')

        def do():
            try:
                robot = RobotArmController(port=port)
                if robot.connect():
                    # Sync local angles with actual robot
                    actual = robot.get_current_angles()
                    with self._angle_lock:
                        for j in JOINTS:
                            self.angles[j] = actual[j]
                        self.gripper = actual['g']
                    self.robot = robot
                    def ok():
                        self.conn_lbl.config(text="🟢 Connected", foreground='green')
                        self.conn_btn.config(text="Disconnect Robot", state='normal')
                        self._log(f"Robot connected on {port}.")
                    self.root.after(0, ok)
                else:
                    def fail():
                        self.conn_lbl.config(text="⚫ Failed", foreground='red')
                        self.conn_btn.config(text="Connect Robot", state='normal')
                        self._log(f"✗ Could not connect on {port}.")
                    self.root.after(0, fail)
            except Exception as e:
                msg = str(e)
                def err():
                    self.conn_lbl.config(text="⚫ Error", foreground='red')
                    self.conn_btn.config(text="Connect Robot", state='normal')
                    self._log(f"✗ {msg}")
                self.root.after(0, err)

        threading.Thread(target=do, daemon=True).start()

    # ═════════════════════════════════════════════════════ UDP listener ════════
    def _start_udp_listener(self):
        self._udp_running = True
        threading.Thread(target=self._udp_loop, daemon=True).start()

    def _udp_loop(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', UDP_PORT))
            sock.settimeout(1.0)
            self.root.after(0, lambda: self.udp_lbl.config(
                text=f"🟢 Listening on :{UDP_PORT}", foreground='green'))
            self._log(f"UDP listening on port {UDP_PORT}")
        except Exception as e:
            self.root.after(0, lambda: self.udp_lbl.config(
                text=f"⚫ UDP error: {e}", foreground='red'))
            self._log(f"✗ UDP bind error: {e}")
            return

        while self._udp_running:
            try:
                data, addr = sock.recvfrom(256)
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

    # ═══════════════════════════════════════════════════ control loop ══════════
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
        # Read joystick (zero if stale)
        now = time.monotonic()
        with self._joy_lock:
            if now - self._last_joy_t > JOY_TIMEOUT:
                axes = {a: 0.0 for a in AXES}
            else:
                axes = dict(self._joy)
            pkts = self._packets

        speed = self.speed_var.get()
        changed = False

        with self._angle_lock:
            new_angles = dict(self.angles)
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

            if changed:
                self.angles = new_angles

        if changed and self.robot is not None:
            try:
                cmd = {j: int(round(new_angles[j])) for j in JOINTS}
                cmd['g'] = self.gripper
                self.robot._stream_command(cmd)
            except Exception:
                pass

        # Update GUI (throttled to ~15 fps via after)
        self.root.after(0, lambda a=dict(new_angles), p=pkts: self._update_display(a, p))

    # ═════════════════════════════════════════════════════ gripper / home ══════
    def _toggle_gripper(self):
        self.gripper = G_OPEN if self.gripper == G_CLOSE else G_CLOSE
        label  = "OPEN"  if self.gripper == G_OPEN  else "CLOSED"
        colour = 'green' if self.gripper == G_OPEN  else 'red'
        btn_t  = "🤚 Open Gripper" if self.gripper == G_CLOSE else "✊ Close Gripper"
        self.grip_lbl.config(text=label, foreground=colour)
        self.grip_btn.config(text=btn_t)
        if self.robot is not None:
            with self._angle_lock:
                cmd = {j: int(round(self.angles[j])) for j in JOINTS}
            cmd['g'] = self.gripper
            threading.Thread(target=self.robot._stream_command,
                             args=(cmd,), daemon=True).start()

    def _go_home(self):
        if self.robot is None:
            return
        def do():
            self._log("Going home…")
            self.robot.home_position(smooth=True, display_progress=False)
            with self._angle_lock:
                for j in JOINTS:
                    self.angles[j] = HOME[j]
                self.gripper = HOME['g']
            self._log("✓ Home reached.")
        threading.Thread(target=do, daemon=True).start()

    # ═════════════════════════════════════════════════════ display update ══════
    def _update_display(self, angles, pkts):
        with self._joy_lock:
            axes = dict(self._joy)
            stale = (time.monotonic() - self._last_joy_t) > JOY_TIMEOUT

        for ax in AXES:
            v = 0.0 if stale else axes[ax]
            # Map -1..+1 → 0..200 for progress bar (100 = centre)
            self._axis_bars[ax]['value'] = int((v + 1.0) * 100)
            self._axis_vals[ax].config(text=f"{v:+.3f}")

        for j in JOINTS:
            a = int(round(angles.get(j, self.angles.get(j, 0))))
            self._joint_bars[j]['value'] = a
            self._joint_lbls[j].config(text=f"{a:3d}°")

        self.pkt_lbl.config(text=f"pkts: {pkts}")

        if stale and pkts > 0:
            self.udp_lbl.config(text="🟡 Joystick silent…", foreground='orange')
        elif pkts > 0:
            self.udp_lbl.config(text=f"🟢 Receiving  :{UDP_PORT}", foreground='green')

    def _update_speed_label(self):
        self.spd_lbl.config(text=f"{self.speed_var.get():.0f} °/s")

    # ═══════════════════════════════════════════════════════════════ helpers ═══
    def _log(self, msg):
        def _do():
            self.log.configure(state='normal')
            self.log.insert(tk.END, msg + "\n")
            self.log.see(tk.END)
            self.log.configure(state='disabled')
        self.root.after(0, _do)

    def on_closing(self):
        self._udp_running     = False
        self._control_running = False
        if self.robot:
            try:
                self.robot.disconnect()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app  = JoystickController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
