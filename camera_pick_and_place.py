#!/usr/bin/env python3
"""
Camera-Guided Pick and Place

  1. Click an object in the camera feed  → arm picks it up
  2. Click where to drop it              → arm places it there

Requires:
  - camera_calibration.json (run camera_calibration.py first)
  - ESP32-CAM streaming
  - Robot connected via serial
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import numpy as np
import cv2
from PIL import Image, ImageTk
import requests

from raspberry_pi_controller import RobotArmController
from camera_calibration import load_calibration, pixel_to_robot

DISPLAY_W = 680
DISPLAY_H = 510

# Workflow states
STATE_IDLE        = "idle"
STATE_WAIT_PICK   = "wait_pick"
STATE_PICKING     = "picking"
STATE_WAIT_PLACE  = "wait_place"
STATE_PLACING     = "placing"


class CameraPickAndPlace:

    def __init__(self, root):
        self.root = root
        self.root.title("Prime-Arm — Camera Pick & Place")
        self.root.geometry("1060x660")
        self.root.resizable(True, True)

        # ── Runtime state ──────────────────────────────────────────────────
        self.robot       = None
        self.streaming   = False
        self.homography  = None
        self.state       = STATE_IDLE

        self.pick_pixel  = None   # (px, py) frame coords
        self.pick_xy     = None   # (rx, ry) robot coords
        self.place_pixel = None
        self.place_xy    = None

        self.latest_frame = None
        self.frame_lock   = threading.Lock()
        self._scale  = 1.0
        self._off_x  = 0
        self._off_y  = 0

        self._build_ui()
        self._load_calib()
        self._schedule_display()
        self._set_state(STATE_IDLE)

    # ══════════════════════════════════════════════════════════════════ UI ═══
    def _build_ui(self):
        self.root.columnconfigure(0, weight=3)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # ── Left: camera ───────────────────────────────────────────────────
        left = ttk.Frame(self.root, padding=8)
        left.grid(row=0, column=0, sticky='nsew')
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)

        # Camera connection bar
        cam_f = ttk.LabelFrame(left, text="Camera", padding=6)
        cam_f.grid(row=0, column=0, sticky='ew', pady=(0, 6))

        ttk.Label(cam_f, text="IP:").pack(side=tk.LEFT)
        self.cam_ip = tk.StringVar(value="192.168.1.100")
        ttk.Entry(cam_f, textvariable=self.cam_ip, width=14).pack(side=tk.LEFT, padx=4)
        ttk.Label(cam_f, text="Port:").pack(side=tk.LEFT)
        self.cam_port = tk.StringVar(value="80")
        ttk.Entry(cam_f, textvariable=self.cam_port, width=5).pack(side=tk.LEFT, padx=4)
        self.cam_btn = ttk.Button(cam_f, text="Connect Cam",
                                  command=self._toggle_stream)
        self.cam_btn.pack(side=tk.LEFT, padx=6)
        self.led_on = False
        self.led_btn = ttk.Button(cam_f, text="💡 LED On",
                                  command=self._toggle_led)
        self.led_btn.pack(side=tk.LEFT, padx=(0, 6))
        self.cam_status = ttk.Label(cam_f, text="⚫ No stream", foreground='red',
                                    font=('Arial', 9, 'bold'))
        self.cam_status.pack(side=tk.LEFT, padx=4)

        # Canvas
        self.canvas_f = ttk.LabelFrame(left,
            text="Camera Feed  —  start a new operation below", padding=4)
        canvas_f = self.canvas_f
        canvas_f.grid(row=1, column=0, sticky='nsew')
        canvas_f.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_f, bg='#111',
                                width=DISPLAY_W, height=DISPLAY_H,
                                cursor='crosshair')
        self.canvas.grid(row=0, column=0, sticky='nsew')
        self.canvas.bind('<Button-1>', self._on_click)
        self.canvas.bind('<Motion>',   self._on_hover)

        self.coord_label = ttk.Label(left, text="Hover over image to preview robot coords",
                                     font=('Consolas', 9), foreground='gray')
        self.coord_label.grid(row=2, column=0, sticky='w', pady=(2, 0))

        # ── Right: controls ────────────────────────────────────────────────
        right = ttk.Frame(self.root, padding=8)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)

        # Robot connection
        rob_f = ttk.LabelFrame(right, text="Robot", padding=6)
        rob_f.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        ttk.Label(rob_f, text="Port:").pack(side=tk.LEFT)
        import sys
        self.rob_port = tk.StringVar(
            value='COM4' if sys.platform.startswith('win') else '/dev/serial0')
        ttk.Entry(rob_f, textvariable=self.rob_port, width=8).pack(side=tk.LEFT, padx=4)
        self.rob_btn = ttk.Button(rob_f, text="Connect", command=self._connect_robot)
        self.rob_btn.pack(side=tk.LEFT, padx=4)
        self.rob_status = ttk.Label(rob_f, text="⚫ Disconnected", foreground='red',
                                    font=('Arial', 9, 'bold'))
        self.rob_status.pack(side=tk.LEFT, padx=4)

        # Calibration
        calib_f = ttk.LabelFrame(right, text="Calibration", padding=6)
        calib_f.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        self.calib_label = ttk.Label(calib_f, text="Loading…", foreground='gray',
                                     font=('Arial', 9), wraplength=200)
        self.calib_label.pack(anchor='w')

        # Settings
        s = ttk.LabelFrame(right, text="Settings", padding=6)
        s.grid(row=2, column=0, sticky='ew', pady=(0, 8))
        s.columnconfigure(1, weight=1)

        def row(parent, r, label, var, from_, to, inc, fmt=None, suffix=""):
            ttk.Label(parent, text=label).grid(row=r, column=0, sticky='e', pady=2)
            kw = dict(from_=from_, to=to, increment=inc, textvariable=var, width=7)
            if fmt:
                kw['format'] = fmt
            ttk.Spinbox(parent, **kw).grid(row=r, column=1, sticky='w', padx=4)
            if suffix:
                ttk.Label(parent, text=suffix, foreground='gray',
                          font=('Arial', 8)).grid(row=r, column=2, sticky='w')

        self.obj_z_var    = tk.DoubleVar(value=0.5)
        self.approach_var = tk.DoubleVar(value=5.0)
        self.dt_var       = tk.DoubleVar(value=0.07)
        self.vdt_var      = tk.DoubleVar(value=0.14)
        self.steps_var    = tk.IntVar(value=30)
        self.g_open_var   = tk.IntVar(value=30)
        self.g_close_var  = tk.IntVar(value=80)

        row(s, 0, "Grasp Z (cm):",        self.obj_z_var,    0.0, 10.0, 0.1,  suffix="= obj_height/2  (e.g. 0.5 for 1 cm obj)")
        row(s, 1, "Approach height (cm):", self.approach_var, 1.0, 20.0, 0.5)
        row(s, 2, "Transit dt (s):",    self.dt_var,       0.01, 0.30, 0.01, fmt="%.2f")
        row(s, 3, "Vertical dt (s):",   self.vdt_var,      0.01, 0.40, 0.01, fmt="%.2f", suffix="descend/retract")
        row(s, 4, "Steps/segment:",     self.steps_var,    5,    100,  5)
        row(s, 5, "Gripper open (°):",  self.g_open_var,   10,   80,   1)
        row(s, 6, "Gripper close (°):", self.g_close_var,  30,   80,   1)

        # Workflow status banner
        self.banner_var = tk.StringVar(value="Connect robot to begin")
        banner = tk.Label(right, textvariable=self.banner_var,
                          font=('Arial', 11, 'bold'), fg='white', bg='#444',
                          wraplength=220, justify='center', pady=8)
        banner.grid(row=3, column=0, sticky='ew', pady=(0, 6))
        self.banner = banner

        # Workflow buttons
        wf_f = ttk.Frame(right)
        wf_f.grid(row=4, column=0, sticky='ew', pady=(0, 8))
        wf_f.columnconfigure(0, weight=1)
        wf_f.columnconfigure(1, weight=1)

        self.start_btn = ttk.Button(wf_f, text="▶  New Pick & Place",
                                    command=self._start_workflow)
        self.start_btn.grid(row=0, column=0, columnspan=2, sticky='ew', pady=2)

        self.cancel_btn = ttk.Button(wf_f, text="✕  Cancel",
                                     command=self._cancel_workflow, state='disabled')
        self.cancel_btn.grid(row=1, column=0, sticky='ew', pady=2, padx=(0, 2))

        self.home_btn = ttk.Button(wf_f, text="🏠 Home",
                                   command=self._go_home)
        self.home_btn.grid(row=1, column=1, sticky='ew', pady=2, padx=(2, 0))

        # Point summary
        pts_f = ttk.LabelFrame(right, text="Selected Points", padding=6)
        pts_f.grid(row=5, column=0, sticky='ew', pady=(0, 8))
        self.pick_label  = ttk.Label(pts_f, text="Pick:   —", font=('Consolas', 9))
        self.pick_label.pack(anchor='w')
        self.place_label = ttk.Label(pts_f, text="Place:  —", font=('Consolas', 9))
        self.place_label.pack(anchor='w')

        # Log
        log_f = ttk.LabelFrame(right, text="Log", padding=4)
        log_f.grid(row=6, column=0, sticky='nsew')
        right.rowconfigure(6, weight=1)
        log_f.columnconfigure(0, weight=1)
        log_f.rowconfigure(0, weight=1)
        self.log = scrolledtext.ScrolledText(log_f, height=7, font=('Consolas', 8),
                                              state='disabled', wrap=tk.WORD)
        self.log.grid(row=0, column=0, sticky='nsew')

    # ══════════════════════════════════════════════════════ state machine ══════
    def _set_state(self, new_state):
        self.state = new_state

        cfg = {
            STATE_IDLE: dict(
                banner="Ready.\nClick '▶ New Pick & Place' to begin.",
                banner_bg='#444',
                canvas_label="Camera Feed",
                start=True, cancel=False,
                cursor='arrow',
            ),
            STATE_WAIT_PICK: dict(
                banner="Step 1 of 2\nClick the OBJECT to pick up.",
                banner_bg='#1a6e2a',
                canvas_label="📦  CLICK THE OBJECT  to pick up",
                start=False, cancel=True,
                cursor='crosshair',
            ),
            STATE_PICKING: dict(
                banner="Picking up object…\nPlease wait.",
                banner_bg='#1a4e8a',
                canvas_label="Camera Feed  —  arm is moving…",
                start=False, cancel=False,
                cursor='watch',
            ),
            STATE_WAIT_PLACE: dict(
                banner="Step 2 of 2\nClick WHERE TO DROP the object.",
                banner_bg='#7a4e00',
                canvas_label="📍  CLICK DROP LOCATION",
                start=False, cancel=True,
                cursor='crosshair',
            ),
            STATE_PLACING: dict(
                banner="Placing object…\nPlease wait.",
                banner_bg='#1a4e8a',
                canvas_label="Camera Feed  —  arm is moving…",
                start=False, cancel=False,
                cursor='watch',
            ),
        }

        c = cfg[new_state]
        self.banner_var.set(c['banner'])
        self.banner.config(bg=c['banner_bg'])
        self.canvas_f.configure(text=c['canvas_label'])
        self.canvas.config(cursor=c['cursor'])
        self.start_btn.config(state='normal' if c['start'] else 'disabled')
        self.cancel_btn.config(state='normal' if c['cancel'] else 'disabled')

    def _start_workflow(self):
        if not self.robot:
            messagebox.showwarning("Not connected", "Connect the robot first.")
            return
        if self.homography is None:
            messagebox.showwarning("No calibration",
                                   "Run camera_calibration.py first.")
            return
        self.pick_pixel  = None
        self.pick_xy     = None
        self.place_pixel = None
        self.place_xy    = None
        self.pick_label.config(text="Pick:   —")
        self.place_label.config(text="Place:  —")
        self._set_state(STATE_WAIT_PICK)
        self._log("\n── New pick & place ──")
        self._log("Click the object you want to pick up.")

    def _cancel_workflow(self):
        self._set_state(STATE_IDLE)
        self._log("Operation cancelled.")

    # ══════════════════════════════════════════════════════════ canvas events ══
    def _canvas_to_frame(self, cx, cy):
        return (cx - self._off_x) / self._scale, (cy - self._off_y) / self._scale

    def _on_hover(self, event):
        fx, fy = self._canvas_to_frame(event.x, event.y)
        if self.homography is not None:
            coords = pixel_to_robot(fx, fy, self.homography)
            if coords:
                self.coord_label.config(
                    text=f"({coords[0]:.1f}, {coords[1]:.1f}) cm  "
                         f"[px {fx:.0f}, {fy:.0f}]",
                    foreground='gray')
                return
        self.coord_label.config(text=f"px ({fx:.0f}, {fy:.0f})", foreground='gray')

    def _on_click(self, event):
        if self.state not in (STATE_WAIT_PICK, STATE_WAIT_PLACE):
            return
        if self.homography is None:
            return

        fx, fy = self._canvas_to_frame(event.x, event.y)
        coords = pixel_to_robot(fx, fy, self.homography)
        if coords is None:
            return
        rx, ry = coords

        if self.state == STATE_WAIT_PICK:
            self.pick_pixel = (fx, fy)
            self.pick_xy    = (rx, ry)
            self.pick_label.config(text=f"Pick:   ({rx:.1f}, {ry:.1f}) cm")
            self._log(f"Pick selected: ({rx:.1f}, {ry:.1f}) cm")
            # Execute pick then transition
            self._set_state(STATE_PICKING)
            threading.Thread(target=self._do_pick, daemon=True).start()

        elif self.state == STATE_WAIT_PLACE:
            self.place_pixel = (fx, fy)
            self.place_xy    = (rx, ry)
            self.place_label.config(text=f"Place:  ({rx:.1f}, {ry:.1f}) cm")
            self._log(f"Place selected: ({rx:.1f}, {ry:.1f}) cm")
            self._set_state(STATE_PLACING)
            threading.Thread(target=self._do_place, daemon=True).start()

    # ══════════════════════════════════════════════════════════ robot actions ══
    def _do_pick(self):
        """
        Move to the pick position, close gripper, retract.
        Manually streams only the 3 pick phases:
          approach → descend → grab → retract
        Does NOT run the place phases (no double-dip).
        """
        rx, ry = self.pick_xy
        obj_z    = self.obj_z_var.get()
        approach = self.approach_var.get()
        dt       = self.dt_var.get()
        vdt      = self.vdt_var.get()
        steps    = self.steps_var.get()
        g_open   = self.g_open_var.get()
        g_close  = self.g_close_var.get()

        self._log(f"Picking at ({rx:.1f}, {ry:.1f}, {obj_z:.1f}) cm …")

        try:
            from inverse_kinematics_servo import calculate_ik
            import time

            current = self.robot.get_current_angles()
            twist   = current['t']
            guess   = {k: current[k] for k in ('b', 's', 'e', 'w')}

            def solve(pos):
                nonlocal guess
                angles, ok, err = calculate_ik(*pos, guess)
                guess = angles
                return angles

            j_current    = {k: current[k] for k in ('b', 's', 'e', 'w')}
            j_above_pick = solve((rx, ry, obj_z + approach))
            j_at_pick    = solve((rx, ry, obj_z))

            def stream_seg(from_j, to_j, g_angle, seg_dt):
                for step in range(1, steps + 1):
                    t = step / steps
                    cmd = {k: int(round(from_j[k] + t * (to_j[k] - from_j[k])))
                           for k in ('b', 's', 'e', 'w')}
                    cmd['t'] = twist
                    cmd['g'] = g_angle
                    self.robot._stream_command(cmd)
                    time.sleep(seg_dt)

            self._log("  → Approaching above pick…")
            stream_seg(j_current, j_above_pick, g_open, dt)

            self._log("  → Descending to pick…")
            stream_seg(j_above_pick, j_at_pick, g_open, vdt)

            self._log("  → Closing gripper…")
            self.robot._stream_command({**j_at_pick, 't': twist, 'g': g_close})
            time.sleep(0.6)

            self._log("  → Retracting…")
            stream_seg(j_at_pick, j_above_pick, g_close, vdt)

            with self.robot.feedback_lock:
                self.robot.current_angles.update({**j_above_pick, 't': twist, 'g': g_close})

            self._log("✓ Object grabbed — now click where to drop it.")
            self.root.after(0, lambda: self._set_state(STATE_WAIT_PLACE))

        except Exception as e:
            self._log(f"✗ Pick error: {e}")
            self.root.after(0, lambda: self._set_state(STATE_IDLE))

    def _do_place(self):
        """
        Carry object from current holding position to the place XY and release.
        """
        px_r, py_r = self.pick_xy
        lx, ly     = self.place_xy
        obj_z      = self.obj_z_var.get()
        approach   = self.approach_var.get()
        dt         = self.dt_var.get()
        vdt        = self.vdt_var.get()
        steps      = self.steps_var.get()
        g_open     = self.g_open_var.get()
        g_close    = self.g_close_var.get()

        self._log(f"Placing at ({lx:.1f}, {ly:.1f}, {obj_z:.1f}) cm …")

        try:
            # At this point the arm is already above the pick point holding the object.
            # We do a pick-and-place where:
            #   pick  = current holding position (above_pick height, gripper already closed)
            #   place = the user's drop location
            # We cheat by setting approach_height=0 so it goes straight to "at_pick"
            # without descending again (arm is already holding).
            #
            # Actually simpler: transfer from above_pick → above_place → descend → release
            # We call execute_pick_and_place with pick_pos = (above_pick XY, obj_z at that height)
            # and gripper already closed, so we skip the re-grab.
            # The cleanest way: move_to_position to above the place, then descend + open.

            # Step 1: Move from above-pick to above-place (carrying object)
            # Step 2: Descend to place height
            # Step 3: Open gripper
            # Step 4: Retract

            above_z = obj_z + approach

            from inverse_kinematics_servo import calculate_ik
            import time

            current = self.robot.get_current_angles()
            twist   = current['t']
            guess   = {k: current[k] for k in ('b', 's', 'e', 'w')}

            def solve(pos):
                nonlocal guess
                angles, ok, err = calculate_ik(*pos, guess)
                guess = angles
                return angles

            j_above_place = solve((lx, ly, obj_z + approach))
            j_at_place    = solve((lx, ly, obj_z))

            j_current = {k: current[k] for k in ('b', 's', 'e', 'w')}

            def stream_seg(from_j, to_j, g_angle, seg_dt):
                for step in range(1, steps + 1):
                    t = step / steps
                    cmd = {k: int(round(from_j[k] + t * (to_j[k] - from_j[k])))
                           for k in ('b', 's', 'e', 'w')}
                    cmd['t'] = twist
                    cmd['g'] = g_angle
                    self.robot._stream_command(cmd)
                    time.sleep(seg_dt)

            self._log("  → Transfer to above place…")
            stream_seg(j_current,    j_above_place, g_close, dt)

            self._log("  → Descend to place…")
            stream_seg(j_above_place, j_at_place,   g_close, vdt)

            self._log("  → Opening gripper…")
            self.robot._stream_command({**j_at_place, 't': twist, 'g': g_open})
            time.sleep(0.8)

            self._log("  → Retracting…")
            stream_seg(j_at_place, j_above_place, g_open, vdt)

            self._log("  → Closing gripper…")
            self.robot._stream_command({**j_above_place, 't': twist, 'g': g_close})
            time.sleep(0.6)

            with self.robot.feedback_lock:
                self.robot.current_angles.update({**j_above_place, 't': twist, 'g': g_close})

            self._log("  → Returning home…")
            self.robot.home_position(smooth=True, display_progress=False)
            self._log("✓ Placed and home!")
            self.root.after(0, lambda: self._set_state(STATE_IDLE))

        except Exception as e:
            self._log(f"✗ Place error: {e}")
            self.root.after(0, lambda: self._set_state(STATE_IDLE))

    def _go_home(self):
        if not self.robot or self.state in (STATE_PICKING, STATE_PLACING):
            return

        def do():
            self._log("🏠 Going home…")
            self.robot.home_position(smooth=True, display_progress=False)
            self._log("✓ Home reached.")

        threading.Thread(target=do, daemon=True).start()

    # ══════════════════════════════════════════════════════ camera / display ══
    def _toggle_led(self):
        ip, port = self.cam_ip.get().strip(), self.cam_port.get().strip()
        endpoint = 'off' if self.led_on else 'on'
        url = f"http://{ip}:81/led/{endpoint}"
        def do():
            try:
                requests.get(url, timeout=3)
                self.led_on = not self.led_on
                label = "💡 LED Off" if self.led_on else "💡 LED On"
                self.root.after(0, lambda: self.led_btn.config(text=label))
            except Exception as e:
                self._log(f"LED error: {e}")
        threading.Thread(target=do, daemon=True).start()

    def _toggle_stream(self):
        if self.streaming:
            self.streaming = False
            self.cam_btn.config(text="Connect Cam")
            self.cam_status.config(text="⚫ No stream", foreground='red')
        else:
            ip, port = self.cam_ip.get().strip(), self.cam_port.get().strip()
            url = f"http://{ip}:{port}/"
            self.streaming = True
            self.cam_btn.config(text="Disconnect Cam")
            self.cam_status.config(text="🟡 Connecting…", foreground='orange')
            threading.Thread(target=self._stream_loop, args=(url,), daemon=True).start()

    def _stream_loop(self, url):
        try:
            resp = requests.get(url, stream=True, timeout=10)
            resp.raise_for_status()
            self.root.after(0, lambda: self.cam_status.config(
                text="🟢 Streaming", foreground='green'))
            self._log(f"Camera connected: {url}")
            buf = bytes()
            for chunk in resp.iter_content(chunk_size=8192):
                if not self.streaming:
                    break
                buf += chunk
                while True:
                    s = buf.find(b'\xff\xd8')
                    e = buf.find(b'\xff\xd9')
                    if s == -1 or e == -1 or e < s:
                        break
                    frame = cv2.imdecode(
                        np.frombuffer(buf[s:e+2], dtype=np.uint8), cv2.IMREAD_COLOR)
                    buf = buf[e+2:]
                    if frame is not None:
                        with self.frame_lock:
                            self.latest_frame = frame
        except requests.exceptions.ConnectionError:
            self._log(f"✗ Camera: connection refused ({url})")
        except Exception as e:
            self._log(f"✗ Camera error: {e}")
        finally:
            self.streaming = False
            self.root.after(0, lambda: [
                self.cam_btn.config(text="Connect Cam"),
                self.cam_status.config(text="⚫ Disconnected", foreground='red'),
            ])

    def _schedule_display(self):
        self._update_display()
        self.root.after(33, self._schedule_display)

    def _update_display(self):
        with self.frame_lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None

        cw = max(self.canvas.winfo_width(),  DISPLAY_W)
        ch = max(self.canvas.winfo_height(), DISPLAY_H)

        if frame is None:
            self.canvas.delete("all")
            self.canvas.create_rectangle(0, 0, cw, ch, fill='#111', outline='')
            self.canvas.create_text(cw//2, ch//2, text="No camera feed",
                                    fill='gray', font=('Arial', 12))
            return

        display = frame.copy()
        self._draw_overlay(display)

        fh, fw  = display.shape[:2]
        scale   = min(cw / fw, ch / fh)
        nw, nh  = int(fw * scale), int(fh * scale)
        off_x   = (cw - nw) // 2
        off_y   = (ch - nh) // 2
        self._scale, self._off_x, self._off_y = scale, off_x, off_y

        rgb = cv2.cvtColor(cv2.resize(display, (nw, nh)), cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, cw, ch, fill='#111', outline='')
        self.canvas.create_image(off_x, off_y, anchor='nw', image=img)
        self.canvas._img_ref = img

    def _draw_overlay(self, frame):
        # Grid
        if self.homography is not None:
            self._draw_grid(frame)

        # Pick marker
        if self.pick_pixel is not None:
            px, py = int(self.pick_pixel[0]), int(self.pick_pixel[1])
            rx, ry = self.pick_xy
            color  = (0, 220, 80)
            cv2.circle(frame, (px, py), 14, color, 2)
            cv2.circle(frame, (px, py), 4,  color, -1)
            cv2.putText(frame, f"PICK ({rx:.1f},{ry:.1f})",
                        (px + 16, py - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)

        # Place marker
        if self.place_pixel is not None:
            px, py = int(self.place_pixel[0]), int(self.place_pixel[1])
            rx, ry = self.place_xy
            color  = (0, 120, 255)
            cv2.circle(frame, (px, py), 14, color, 2)
            cv2.circle(frame, (px, py), 4,  color, -1)
            cv2.putText(frame, f"PLACE ({rx:.1f},{ry:.1f})",
                        (px + 16, py - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)

        # Instruction overlay when waiting for a click
        if self.state == STATE_WAIT_PICK:
            self._draw_instruction(frame, "CLICK THE OBJECT TO PICK UP", (0, 220, 80))
        elif self.state == STATE_WAIT_PLACE:
            self._draw_instruction(frame, "CLICK WHERE TO DROP IT", (0, 120, 255))

    def _draw_instruction(self, frame, text, color):
        fh, fw = frame.shape[:2]
        # Semi-transparent dark bar at bottom
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, fh - 36), (fw, fh), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        cv2.putText(frame, text, (fw // 2 - len(text) * 5, fh - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)

    def _draw_grid(self, frame, step=5, extent=25):
        fh, fw = frame.shape[:2]
        try:
            H_inv = np.linalg.inv(self.homography)
        except np.linalg.LinAlgError:
            return

        def r2p(rx, ry):
            p = H_inv @ np.array([rx, ry, 1.0])
            if abs(p[2]) < 1e-9:
                return None
            ix, iy = int(p[0] / p[2]), int(p[1] / p[2])
            return (ix, iy) if 0 <= ix < fw and 0 <= iy < fh else None

        dense = range(-extent, extent + 1, 2)
        for x in range(-extent, extent + 1, step):
            pts = [r2p(x, y) for y in dense]
            pts = [p for p in pts if p]
            col = (0, 200, 255) if x == 0 else (55, 55, 55)
            for j in range(len(pts) - 1):
                cv2.line(frame, pts[j], pts[j+1], col, 1 + (x == 0), cv2.LINE_AA)
        for y in range(-extent, extent + 1, step):
            pts = [r2p(x, y) for x in dense]
            pts = [p for p in pts if p]
            col = (0, 200, 255) if y == 0 else (55, 55, 55)
            for j in range(len(pts) - 1):
                cv2.line(frame, pts[j], pts[j+1], col, 1 + (y == 0), cv2.LINE_AA)

    # ══════════════════════════════════════════════════ robot connection ══════
    def _connect_robot(self):
        def do():
            self.root.after(0, lambda: self.rob_status.config(
                text="🟡 Connecting…", foreground='orange'))
            try:
                if self.robot:
                    self.robot.disconnect()
                self.robot = RobotArmController(port=self.rob_port.get().strip())
                if self.robot.connect():
                    pos = self.robot.get_end_effector_position()
                    self.root.after(0, lambda: self.rob_status.config(
                        text="🟢 Connected", foreground='green'))
                    self._log(f"Robot connected. EE: ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})")
                else:
                    self.robot = None
                    self.root.after(0, lambda: self.rob_status.config(
                        text="🔴 Failed", foreground='red'))
                    self._log("✗ Robot connection failed.")
            except Exception as e:
                self.robot = None
                self.root.after(0, lambda: self.rob_status.config(
                    text="🔴 Error", foreground='red'))
                self._log(f"✗ Robot error: {e}")
        threading.Thread(target=do, daemon=True).start()

    # ══════════════════════════════════════════════════════════════ helpers ══
    def _load_calib(self):
        self.homography = load_calibration()
        if self.homography is not None:
            self.calib_label.config(text="✓ Calibration loaded", foreground='green')
            self._log("Calibration loaded.")
        else:
            self.calib_label.config(
                text="✗ No calibration.\nRun camera_calibration.py first.",
                foreground='red')
            self._log("⚠ No calibration found.")

    def _log(self, msg):
        def _do():
            self.log.configure(state='normal')
            self.log.insert(tk.END, msg + "\n")
            self.log.see(tk.END)
            self.log.configure(state='disabled')
        self.root.after(0, _do)

    def on_closing(self):
        self.streaming = False
        if self.robot:
            self.robot.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = CameraPickAndPlace(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
