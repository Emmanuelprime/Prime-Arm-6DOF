#!/usr/bin/env python3
"""
ESP32-CAM Calibration Tool for Prime-Arm Robot

Maps overhead camera pixel coordinates → robot workspace XY (cm) using a
perspective homography so you can click objects in the image and get their
robot coordinates directly.

Workflow:
  1. Enter the ESP32-CAM IP and click Connect.
  2. Switch to Calibration mode.
  3. Place a marker at a known robot (X, Y) position on the table.
  4. Click the marker in the image  → its pixel coords auto-fill.
  5. Type the robot X, Y (cm) and click "Add Point".
  6. Repeat for 4-8 spread-out points (more = better accuracy).
  7. Click "Compute Calibration" — RMS error (cm) is shown.
  8. Click "Save" → writes camera_calibration.json next to this file.

  In Test mode:
    - Hover to see live robot coordinates under the cursor.
    - Click to print/log the robot position.

  The calibration JSON can be loaded by other scripts:
    from camera_calibration import load_calibration, pixel_to_robot
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import json
import os
import time
from datetime import datetime

try:
    import numpy as np
    import cv2
    from PIL import Image, ImageTk
except ImportError as e:
    import sys
    print(f"Missing dependency: {e}")
    print("Install with: pip install opencv-python numpy pillow requests")
    sys.exit(1)

try:
    import requests
except ImportError:
    import sys
    print("Missing dependency: requests")
    print("Install with: pip install requests")
    sys.exit(1)

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "camera_calibration.json")


# ─────────────────────────────────────────────────────── public helpers ──────
def load_calibration(path=CALIB_FILE):
    """Load homography from JSON. Returns np.ndarray (3x3) or None."""
    if not os.path.exists(path):
        return None
    with open(path) as f:
        data = json.load(f)
    return np.array(data["homography"], dtype=np.float64)


def pixel_to_robot(px, py, homography):
    """
    Convert a frame pixel coordinate to robot XY (cm).

    Args:
        px, py:     pixel position in the raw camera frame
        homography: 3×3 numpy matrix from load_calibration()

    Returns:
        (rx, ry) tuple in cm, or None if homography is None
    """
    if homography is None:
        return None
    p = homography @ np.array([px, py, 1.0], dtype=np.float64)
    if abs(p[2]) < 1e-9:
        return None
    return float(p[0] / p[2]), float(p[1] / p[2])


# ──────────────────────────────────────────────────────────────── GUI ─────────
class CameraCalibrationTool:

    DISPLAY_W = 640
    DISPLAY_H = 480

    def __init__(self, root):
        self.root = root
        self.root.title("Prime-Arm — Camera Calibration")
        self.root.geometry("1160x780")
        self.root.resizable(True, True)

        # Stream state
        self.streaming     = False
        self.stream_thread = None
        self.latest_frame  = None          # raw numpy BGR
        self.frame_lock    = threading.Lock()

        # Calibration state
        self.calib_points  = []            # [{"pixel":[px,py], "robot":[rx,ry]}, …]
        self.homography    = None          # 3×3 float64  (pixel → robot)
        self.pending_pixel = None          # frame pixel awaiting robot-coord entry
        self.mode          = tk.StringVar(value="calibration")

        # Canvas transform (updated each frame)
        self._scale  = 1.0
        self._off_x  = 0
        self._off_y  = 0
        self._frame_w = self.DISPLAY_W
        self._frame_h = self.DISPLAY_H

        # Auto-calibration state
        self.robot         = None       # RobotArmController when connected
        self.ac_running    = False
        self.ac_pick_mode  = False      # waiting for user to click tip in frame
        self.tip_hsv       = None       # (h_min,h_max,s_min,s_max,v_min,v_max)
        self.tip_color_bgr = None

        self._build_ui()
        self._load_calibration()
        self._schedule_display()

    # ═══════════════════════════════════════════════════════════════ build UI ═
    def _build_ui(self):
        self.root.columnconfigure(0, weight=3)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # ── Left column: video ────────────────────────────────────────────────
        left = ttk.Frame(self.root, padding=8)
        left.grid(row=0, column=0, sticky='nsew')
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)

        # Connection bar
        conn_f = ttk.LabelFrame(left, text="Connection", padding=6)
        conn_f.grid(row=0, column=0, sticky='ew', pady=(0, 6))

        ttk.Label(conn_f, text="ESP32-CAM IP:").pack(side=tk.LEFT)
        self.ip_var = tk.StringVar(value="192.168.1.100")
        ttk.Entry(conn_f, textvariable=self.ip_var, width=16).pack(side=tk.LEFT, padx=4)
        ttk.Label(conn_f, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="80")
        ttk.Entry(conn_f, textvariable=self.port_var, width=6).pack(side=tk.LEFT, padx=4)
        self.conn_btn = ttk.Button(conn_f, text="Connect", command=self._toggle_stream)
        self.conn_btn.pack(side=tk.LEFT, padx=6)
        self.led_on = False
        self.led_btn = ttk.Button(conn_f, text="💡 LED On", command=self._toggle_led)
        self.led_btn.pack(side=tk.LEFT, padx=(0, 6))
        self.conn_label = ttk.Label(conn_f, text="⚫ Disconnected", foreground='red',
                                    font=('Arial', 9, 'bold'))
        self.conn_label.pack(side=tk.LEFT, padx=8)

        # Canvas
        canvas_frame = ttk.LabelFrame(left, text="Camera Feed  (click to place calibration points)", padding=4)
        canvas_frame.grid(row=1, column=0, sticky='nsew')
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, bg='#111',
                                width=self.DISPLAY_W, height=self.DISPLAY_H,
                                cursor='crosshair')
        self.canvas.grid(row=0, column=0, sticky='nsew')
        self.canvas.bind('<Button-1>', self._on_canvas_click)
        self.canvas.bind('<Motion>',   self._on_canvas_hover)

        self.coord_label = ttk.Label(left, text="Hover over image to see robot coordinates",
                                     font=('Consolas', 9), foreground='gray')
        self.coord_label.grid(row=2, column=0, sticky='w', pady=(2, 0))

        # ── Right column: controls ────────────────────────────────────────────
        right = ttk.Frame(self.root, padding=8)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)

        # Mode
        mode_f = ttk.LabelFrame(right, text="Mode", padding=6)
        mode_f.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        ttk.Radiobutton(mode_f, text="Calibration  (click to add points)",
                        variable=self.mode, value="calibration",
                        command=self._on_mode_change).pack(anchor='w')
        ttk.Radiobutton(mode_f, text="Test / Measure  (click to read coords)",
                        variable=self.mode, value="test",
                        command=self._on_mode_change).pack(anchor='w')

        # Add calibration point
        calib_f = ttk.LabelFrame(right, text="Add Calibration Point", padding=6)
        calib_f.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        calib_f.columnconfigure(1, weight=1)

        ttk.Label(calib_f, text="Step 1: click a marker in the image.",
                  font=('Arial', 8), foreground='blue').grid(
                      row=0, column=0, columnspan=4, sticky='w', pady=(0, 4))
        ttk.Label(calib_f, text="Step 2: type its robot coords and click Add.",
                  font=('Arial', 8), foreground='blue').grid(
                      row=1, column=0, columnspan=4, sticky='w', pady=(0, 8))

        ttk.Label(calib_f, text="Pixel X:").grid(row=2, column=0, sticky='e')
        self.pix_x_var = tk.StringVar(value="—")
        ttk.Label(calib_f, textvariable=self.pix_x_var,
                  font=('Consolas', 9), width=8).grid(row=2, column=1, sticky='w', padx=4)
        ttk.Label(calib_f, text="Pixel Y:").grid(row=2, column=2, sticky='e')
        self.pix_y_var = tk.StringVar(value="—")
        ttk.Label(calib_f, textvariable=self.pix_y_var,
                  font=('Consolas', 9), width=8).grid(row=2, column=3, sticky='w', padx=4)

        ttk.Label(calib_f, text="Robot X (cm):").grid(row=3, column=0, sticky='e', pady=4)
        self.rob_x_var = tk.StringVar(value="0")
        ttk.Entry(calib_f, textvariable=self.rob_x_var, width=8).grid(
            row=3, column=1, sticky='w', padx=4)
        ttk.Label(calib_f, text="Robot Y (cm):").grid(row=3, column=2, sticky='e')
        self.rob_y_var = tk.StringVar(value="0")
        ttk.Entry(calib_f, textvariable=self.rob_y_var, width=8).grid(
            row=3, column=3, sticky='w', padx=4)

        add_row = ttk.Frame(calib_f)
        add_row.grid(row=4, column=0, columnspan=4, sticky='ew', pady=(4, 0))
        ttk.Button(add_row, text="Add Point",    command=self._add_calib_point).pack(side=tk.LEFT, padx=2)
        ttk.Button(add_row, text="Remove Last",  command=self._remove_last_point).pack(side=tk.LEFT, padx=2)
        ttk.Button(add_row, text="Clear All",    command=self._clear_points).pack(side=tk.LEFT, padx=2)

        # Points list
        list_f = ttk.LabelFrame(right, text="Collected Points  (need ≥ 4)", padding=4)
        list_f.grid(row=2, column=0, sticky='ew', pady=(0, 8))
        list_f.columnconfigure(0, weight=1)

        self.points_list = tk.Listbox(list_f, height=7, font=('Consolas', 8),
                                      selectmode=tk.SINGLE)
        self.points_list.grid(row=0, column=0, sticky='ew')
        sb = ttk.Scrollbar(list_f, command=self.points_list.yview)
        sb.grid(row=0, column=1, sticky='ns')
        self.points_list.config(yscrollcommand=sb.set)

        # Compute / Save / Load
        action_f = ttk.LabelFrame(right, text="Calibration", padding=6)
        action_f.grid(row=3, column=0, sticky='ew', pady=(0, 8))
        action_f.columnconfigure(0, weight=1)

        self.compute_btn = ttk.Button(action_f, text="Compute Calibration",
                                      command=self._compute_calibration,
                                      state='disabled')
        self.compute_btn.grid(row=0, column=0, columnspan=2, sticky='ew', pady=2)

        self.calib_status = ttk.Label(action_f, text="No calibration loaded",
                                      foreground='red', font=('Arial', 8), wraplength=200)
        self.calib_status.grid(row=1, column=0, columnspan=2, sticky='w', pady=2)

        rms_row = ttk.Frame(action_f)
        rms_row.grid(row=2, column=0, columnspan=2, sticky='w')
        ttk.Label(rms_row, text="RMS error (cm):").pack(side=tk.LEFT)
        self.rms_label = ttk.Label(rms_row, text="—", font=('Consolas', 9, 'bold'))
        self.rms_label.pack(side=tk.LEFT, padx=4)

        save_row = ttk.Frame(action_f)
        save_row.grid(row=3, column=0, sticky='ew', pady=(4, 0))
        ttk.Button(save_row, text="Save", command=self._save_calibration).pack(side=tk.LEFT, padx=2)
        ttk.Button(save_row, text="Load", command=self._load_calibration_dialog).pack(side=tk.LEFT, padx=2)

        # ── Auto Calibration (Robot-Guided) ──────────────────────────────────
        ac_f = ttk.LabelFrame(right, text="Auto Calibration  (Robot-Guided)", padding=6)
        ac_f.grid(row=4, column=0, sticky='ew', pady=(0, 8))
        ac_f.columnconfigure(1, weight=1)

        # Robot connection
        r_row = ttk.Frame(ac_f)
        r_row.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 4))
        ttk.Label(r_row, text="Port:").pack(side=tk.LEFT)
        import sys
        self.ac_port_var = tk.StringVar(
            value='COM4' if sys.platform.startswith('win') else '/dev/serial0')
        ttk.Entry(r_row, textvariable=self.ac_port_var, width=7).pack(side=tk.LEFT, padx=4)
        self.ac_robot_btn = ttk.Button(r_row, text="Connect Robot",
                                       command=self._ac_toggle_robot)
        self.ac_robot_btn.pack(side=tk.LEFT, padx=4)
        self.ac_robot_lbl = ttk.Label(r_row, text="⚫ Not connected",
                                      foreground='red', font=('Arial', 8))
        self.ac_robot_lbl.pack(side=tk.LEFT, padx=4)

        # Grid parameters
        g_f = ttk.LabelFrame(ac_f, text="Grid", padding=4)
        g_f.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(0, 4))

        ttk.Label(g_f, text="Rows:").grid(row=0, column=0, sticky='e')
        self.ac_rows_var = tk.IntVar(value=3)
        ttk.Spinbox(g_f, textvariable=self.ac_rows_var, from_=2, to=6,
                    width=4).grid(row=0, column=1, sticky='w', padx=2)
        ttk.Label(g_f, text="Cols:", ).grid(row=0, column=2, sticky='e', padx=(8, 0))
        self.ac_cols_var = tk.IntVar(value=3)
        ttk.Spinbox(g_f, textvariable=self.ac_cols_var, from_=2, to=6,
                    width=4).grid(row=0, column=3, sticky='w', padx=2)

        ttk.Label(g_f, text="X (cm):").grid(row=1, column=0, sticky='e', pady=2)
        self.ac_xmin_var = tk.DoubleVar(value=-10.0)
        self.ac_xmax_var = tk.DoubleVar(value=10.0)
        ttk.Entry(g_f, textvariable=self.ac_xmin_var, width=6).grid(row=1, column=1, sticky='w', padx=2)
        ttk.Label(g_f, text="→").grid(row=1, column=2, sticky='w', padx=2)
        ttk.Entry(g_f, textvariable=self.ac_xmax_var, width=6).grid(row=1, column=3, sticky='w', padx=2)

        ttk.Label(g_f, text="Y (cm):").grid(row=2, column=0, sticky='e', pady=2)
        self.ac_ymin_var = tk.DoubleVar(value=10.0)
        self.ac_ymax_var = tk.DoubleVar(value=25.0)
        ttk.Entry(g_f, textvariable=self.ac_ymin_var, width=6).grid(row=2, column=1, sticky='w', padx=2)
        ttk.Label(g_f, text="→").grid(row=2, column=2, sticky='w', padx=2)
        ttk.Entry(g_f, textvariable=self.ac_ymax_var, width=6).grid(row=2, column=3, sticky='w', padx=2)

        ttk.Label(g_f, text="Z (cm):").grid(row=3, column=0, sticky='e', pady=2)
        self.ac_z_var = tk.DoubleVar(value=5.0)
        ttk.Entry(g_f, textvariable=self.ac_z_var, width=6).grid(row=3, column=1, sticky='w', padx=2)
        ttk.Label(g_f, text="gripper height above table", foreground='gray',
                  font=('Arial', 8)).grid(row=3, column=2, columnspan=2, sticky='w')

        # Tip colour picker
        tip_row = ttk.Frame(ac_f)
        tip_row.grid(row=2, column=0, columnspan=2, sticky='ew', pady=(0, 4))
        ttk.Label(tip_row, text="Tip colour:").pack(side=tk.LEFT)
        self.ac_color_btn = ttk.Button(tip_row, text="Click tip in frame",
                                       command=self._ac_enter_pick_mode)
        self.ac_color_btn.pack(side=tk.LEFT, padx=4)
        self.ac_swatch = tk.Label(tip_row, width=3, bg='#888', relief='sunken')
        self.ac_swatch.pack(side=tk.LEFT)
        self.ac_color_lbl = ttk.Label(tip_row, text=" not set",
                                      foreground='gray', font=('Arial', 8))
        self.ac_color_lbl.pack(side=tk.LEFT)

        # Settle time
        settle_row = ttk.Frame(ac_f)
        settle_row.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(0, 4))
        ttk.Label(settle_row, text="Settle (s):").pack(side=tk.LEFT)
        self.ac_settle_var = tk.DoubleVar(value=1.5)
        ttk.Spinbox(settle_row, textvariable=self.ac_settle_var,
                    from_=0.5, to=5.0, increment=0.5, width=5,
                    format="%.1f").pack(side=tk.LEFT, padx=4)
        ttk.Label(settle_row, text="wait after each move before capture",
                  foreground='gray', font=('Arial', 8)).pack(side=tk.LEFT)

        # Start + progress
        self.ac_start_btn = ttk.Button(ac_f, text="▶  Start Auto-Calibrate",
                                       command=self._ac_start, state='disabled')
        self.ac_start_btn.grid(row=4, column=0, columnspan=2, sticky='ew', pady=(4, 2))
        self.ac_progress = ttk.Progressbar(ac_f, mode='determinate', maximum=100)
        self.ac_progress.grid(row=5, column=0, columnspan=2, sticky='ew', pady=(0, 2))
        self.ac_status_lbl = ttk.Label(ac_f, text="Connect robot and pick tip colour to begin",
                                       font=('Arial', 8), foreground='gray')
        self.ac_status_lbl.grid(row=6, column=0, columnspan=2, sticky='w')

        # Log
        log_f = ttk.LabelFrame(right, text="Log", padding=4)
        log_f.grid(row=5, column=0, sticky='nsew', pady=(0, 4))
        right.rowconfigure(5, weight=1)
        log_f.columnconfigure(0, weight=1)
        log_f.rowconfigure(0, weight=1)
        self.log = scrolledtext.ScrolledText(log_f, height=8, font=('Consolas', 8),
                                              state='disabled', wrap=tk.WORD)
        self.log.grid(row=0, column=0, sticky='nsew')

    # ═══════════════════════════════════════════════════════════════ streaming ═
    def _toggle_led(self):
        ip, port = self.ip_var.get().strip(), self.port_var.get().strip()
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
            self._stop_stream()
        else:
            self._start_stream()

    def _start_stream(self):
        ip   = self.ip_var.get().strip()
        port = self.port_var.get().strip()
        url  = f"http://{ip}:{port}/"
        self._log(f"Connecting to {url}…")
        self.conn_label.config(text="🟡 Connecting…", foreground='orange')
        self.conn_btn.config(text="Disconnect")
        self.streaming = True
        self.stream_thread = threading.Thread(target=self._stream_loop,
                                              args=(url,), daemon=True)
        self.stream_thread.start()

    def _stop_stream(self):
        self.streaming = False
        self.conn_btn.config(text="Connect")
        self.conn_label.config(text="⚫ Disconnected", foreground='red')
        self._log("Stream stopped.")

    def _stream_loop(self, url):
        """
        Background thread: reads the MJPEG multipart stream from the ESP32-CAM.
        Extracts individual JPEG frames by searching for SOI/EOI markers
        (0xFF 0xD8 … 0xFF 0xD9) which is more robust than parsing HTTP headers.
        """
        try:
            resp = requests.get(url, stream=True, timeout=10)
            resp.raise_for_status()
            self.root.after(0, lambda: self.conn_label.config(
                text="🟢 Streaming", foreground='green'))
            self._log("Stream connected.")

            buf = bytes()
            for chunk in resp.iter_content(chunk_size=8192):
                if not self.streaming:
                    break
                buf += chunk
                # Extract all complete JPEG frames in the buffer
                while True:
                    start = buf.find(b'\xff\xd8')
                    end   = buf.find(b'\xff\xd9')
                    if start == -1 or end == -1 or end < start:
                        break
                    jpg = buf[start:end + 2]
                    buf = buf[end + 2:]
                    frame = cv2.imdecode(
                        np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.frame_lock:
                            self.latest_frame = frame

        except requests.exceptions.ConnectionError:
            self._log(f"✗ Connection refused — is the ESP32-CAM at {url}?")
        except requests.exceptions.Timeout:
            self._log("✗ Connection timed out.")
        except Exception as e:
            self._log(f"✗ Stream error: {e}")
        finally:
            self.streaming = False
            self.root.after(0, lambda: [
                self.conn_btn.config(text="Connect"),
                self.conn_label.config(text="⚫ Disconnected", foreground='red'),
            ])

    # ═══════════════════════════════════════════════════════════════ display ═══
    def _schedule_display(self):
        self._update_display()
        self.root.after(33, self._schedule_display)   # ~30 fps

    def _update_display(self):
        with self.frame_lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None

        cw = max(self.canvas.winfo_width(),  self.DISPLAY_W)
        ch = max(self.canvas.winfo_height(), self.DISPLAY_H)

        if frame is None:
            self.canvas.delete("all")
            self.canvas.create_rectangle(0, 0, cw, ch, fill='#111', outline='')
            self.canvas.create_text(cw // 2, ch // 2,
                                    text="No stream — enter IP and click Connect",
                                    fill='gray', font=('Arial', 12))
            return

        # Draw overlay (calibration markers, grid) on a copy
        display = frame.copy()
        self._draw_overlay(display)

        # Letterbox-fit to canvas
        fh, fw = display.shape[:2]
        scale  = min(cw / fw, ch / fh)
        nw, nh = int(fw * scale), int(fh * scale)
        off_x  = (cw - nw) // 2
        off_y  = (ch - nh) // 2

        self._scale   = scale
        self._off_x   = off_x
        self._off_y   = off_y
        self._frame_w = fw
        self._frame_h = fh

        rgb = cv2.cvtColor(cv2.resize(display, (nw, nh)), cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(Image.fromarray(rgb))

        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, cw, ch, fill='#111', outline='')
        self.canvas.create_image(off_x, off_y, anchor='nw', image=img)
        self.canvas._img_ref = img    # prevent garbage collection

    def _draw_overlay(self, frame):
        """Draw calibration markers and (once calibrated) a robot-space grid."""
        fh, fw = frame.shape[:2]

        # Existing calibration points
        for i, pt in enumerate(self.calib_points):
            px, py = int(pt['pixel'][0]), int(pt['pixel'][1])
            rx, ry = pt['robot']
            cv2.circle(frame, (px, py), 6, (0, 220, 0), 2)
            cv2.circle(frame, (px, py), 2, (0, 220, 0), -1)
            cv2.putText(frame, f"#{i+1} ({rx:.0f},{ry:.0f})",
                        (px + 8, py - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, (0, 220, 0), 1, cv2.LINE_AA)

        # Pending (not yet added) click
        if self.pending_pixel is not None:
            px, py = int(self.pending_pixel[0]), int(self.pending_pixel[1])
            cv2.circle(frame, (px, py), 9, (0, 140, 255), 2)
            cv2.drawMarker(frame, (px, py), (0, 140, 255),
                           markerType=cv2.MARKER_CROSS,
                           markerSize=22, thickness=1)

        # Robot-space grid projected into the image
        if self.homography is not None:
            self._draw_workspace_grid(frame)

    def _draw_workspace_grid(self, frame, step=5, extent=30):
        """
        Project a regular robot XY grid (every `step` cm) back into image space
        using the inverse homography so the user can see the calibrated mapping.
        """
        fh, fw = frame.shape[:2]
        try:
            H_inv = np.linalg.inv(self.homography)
        except np.linalg.LinAlgError:
            return

        def r2p(rx, ry):
            """Robot coords → frame pixel (or None if degenerate/out-of-frame)."""
            p = H_inv @ np.array([rx, ry, 1.0])
            if abs(p[2]) < 1e-9:
                return None
            ix, iy = int(p[0] / p[2]), int(p[1] / p[2])
            if 0 <= ix < fw and 0 <= iy < fh:
                return ix, iy
            return None

        ys = range(-extent, extent + 1, 2)
        xs = range(-extent, extent + 1, 2)

        # Lines of constant X (vertical in robot space)
        for x in range(-extent, extent + 1, step):
            pts = [r2p(x, y) for y in ys]
            pts = [p for p in pts if p is not None]
            color = (0, 180, 255) if x == 0 else (70, 70, 70)
            thick = 2 if x == 0 else 1
            for j in range(len(pts) - 1):
                cv2.line(frame, pts[j], pts[j + 1], color, thick, cv2.LINE_AA)
            if x % 10 == 0 and pts:
                cv2.putText(frame, f"X={x}", pts[0],
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        # Lines of constant Y (horizontal in robot space)
        for y in range(-extent, extent + 1, step):
            pts = [r2p(x, y) for x in xs]
            pts = [p for p in pts if p is not None]
            color = (0, 180, 255) if y == 0 else (70, 70, 70)
            thick = 2 if y == 0 else 1
            for j in range(len(pts) - 1):
                cv2.line(frame, pts[j], pts[j + 1], color, thick, cv2.LINE_AA)
            if y % 10 == 0 and pts:
                cv2.putText(frame, f"Y={y}", pts[0],
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

    # ═══════════════════════════════════════════════════════ coordinate utils ══
    def _canvas_to_frame(self, cx, cy):
        """Convert canvas click/hover pixel → raw frame pixel coordinates."""
        fx = (cx - self._off_x) / self._scale
        fy = (cy - self._off_y) / self._scale
        return fx, fy

    def _frame_pixel_to_robot(self, px, py):
        """Apply forward homography: frame pixel → robot XY (cm)."""
        return pixel_to_robot(px, py, self.homography)

    # ═══════════════════════════════════════════════════════════ canvas events ═
    def _on_canvas_click(self, event):
        fx, fy = self._canvas_to_frame(event.x, event.y)

        # Auto-calibration colour-pick mode — intercept all clicks
        if self.ac_pick_mode:
            self._ac_set_color_from_frame(fx, fy)
            return

        if self.mode.get() == "test":
            coords = self._frame_pixel_to_robot(fx, fy)
            if coords:
                msg = f"Click → robot ({coords[0]:.1f}, {coords[1]:.1f}) cm  [pixel ({fx:.0f}, {fy:.0f})]"
                self._log(msg)
                self.coord_label.config(text=msg, foreground='black')
            else:
                self._log("No calibration loaded — compute calibration first.")
            return

        # Calibration mode: record the clicked pixel
        self.pending_pixel = (fx, fy)
        self.pix_x_var.set(f"{fx:.1f}")
        self.pix_y_var.set(f"{fy:.1f}")
        self._log(f"Pixel selected: ({fx:.1f}, {fy:.1f}) — enter robot XY and click Add Point.")

    def _on_canvas_hover(self, event):
        fx, fy = self._canvas_to_frame(event.x, event.y)
        if self.homography is not None:
            coords = self._frame_pixel_to_robot(fx, fy)
            if coords:
                self.coord_label.config(
                    text=f"Pixel ({fx:.0f}, {fy:.0f})  →  Robot ({coords[0]:.1f}, {coords[1]:.1f}) cm",
                    foreground='gray')
                return
        self.coord_label.config(text=f"Pixel ({fx:.0f}, {fy:.0f})",
                                foreground='gray')

    # ═══════════════════════════════════════════════════════ manage points ═══
    def _add_calib_point(self):
        if self.pending_pixel is None:
            messagebox.showwarning("No pixel selected",
                                   "Click a point on the image first.")
            return
        try:
            rx = float(self.rob_x_var.get())
            ry = float(self.rob_y_var.get())
        except ValueError:
            messagebox.showerror("Invalid input",
                                  "Robot X and Y must be numbers.")
            return

        pt = {"pixel": list(self.pending_pixel), "robot": [rx, ry]}
        self.calib_points.append(pt)
        self.pending_pixel = None
        self.pix_x_var.set("—")
        self.pix_y_var.set("—")
        self._refresh_points_list()
        self._log(f"Added #{len(self.calib_points)}: "
                  f"pixel ({pt['pixel'][0]:.1f}, {pt['pixel'][1]:.1f}) "
                  f"→ robot ({rx:.1f}, {ry:.1f}) cm")
        if len(self.calib_points) >= 4:
            self.compute_btn.config(state='normal')

    def _remove_last_point(self):
        if not self.calib_points:
            return
        removed = self.calib_points.pop()
        self._refresh_points_list()
        self._log(f"Removed #{len(self.calib_points)+1}: "
                  f"pixel ({removed['pixel'][0]:.1f}, {removed['pixel'][1]:.1f})")
        if len(self.calib_points) < 4:
            self.compute_btn.config(state='disabled')

    def _clear_points(self):
        if not self.calib_points:
            return
        if messagebox.askyesno("Clear all", "Remove all calibration points?"):
            self.calib_points.clear()
            self.pending_pixel = None
            self.pix_x_var.set("—")
            self.pix_y_var.set("—")
            self._refresh_points_list()
            self.compute_btn.config(state='disabled')
            self._log("All points cleared.")

    def _refresh_points_list(self):
        self.points_list.delete(0, tk.END)
        for i, pt in enumerate(self.calib_points):
            px, py = pt['pixel']
            rx, ry = pt['robot']
            self.points_list.insert(
                tk.END, f"{i+1:2d}  pix({px:6.1f},{py:6.1f})  rob({rx:6.1f},{ry:6.1f})")

    # ═══════════════════════════════════════════════════ compute calibration ══
    def _compute_calibration(self):
        if len(self.calib_points) < 4:
            messagebox.showwarning("Need ≥ 4 points",
                                   "Add at least 4 spread-out calibration points.")
            return

        src = np.array([pt['pixel'] for pt in self.calib_points], dtype=np.float64)
        dst = np.array([pt['robot']  for pt in self.calib_points], dtype=np.float64)

        # RANSAC-based homography (robust to a bad point or two)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransacReprojThreshold=2.0)
        if H is None:
            messagebox.showerror("Calibration failed",
                                  "Could not compute homography.\n"
                                  "Ensure points are spread across the workspace "
                                  "and not collinear.")
            return

        self.homography = H

        # Per-point reprojection error
        errors = []
        for pt in self.calib_points:
            px, py = pt['pixel']
            rx, ry = pt['robot']
            pred = H @ np.array([px, py, 1.0])
            pred /= pred[2]
            errors.append(np.sqrt((pred[0] - rx)**2 + (pred[1] - ry)**2))

        rms     = float(np.sqrt(np.mean(np.array(errors)**2)))
        inliers = int(mask.sum())
        n       = len(self.calib_points)

        self.rms_label.config(text=f"{rms:.3f} cm  ({inliers}/{n} inliers)")
        self.calib_status.config(
            text=f"✓ Calibrated  ({n} pts, RMS {rms:.2f} cm)",
            foreground='green' if rms < 1.0 else 'orange')
        self._log(f"Calibration OK — RMS {rms:.3f} cm, inliers {inliers}/{n}")

        if rms > 2.0:
            self._log("⚠ High RMS error — consider adding more spread-out points "
                      "or re-clicking inaccurate ones.")

    # ═════════════════════════════════════════════════════════ save / load ════
    def _save_calibration(self):
        if self.homography is None:
            messagebox.showwarning("Not calibrated",
                                   "Compute calibration before saving.")
            return
        data = {
            "homography":         self.homography.tolist(),
            "calibration_points": self.calib_points,
            "timestamp":          datetime.now().isoformat(),
        }
        with open(CALIB_FILE, 'w') as f:
            json.dump(data, f, indent=2)
        self._log(f"Saved → {CALIB_FILE}")
        messagebox.showinfo("Saved", f"Calibration saved to:\n{CALIB_FILE}")

    def _load_calibration(self):
        """Silent load on startup."""
        if not os.path.exists(CALIB_FILE):
            return
        try:
            with open(CALIB_FILE) as f:
                data = json.load(f)
            self.homography   = np.array(data['homography'], dtype=np.float64)
            self.calib_points = data.get('calibration_points', [])
            self._refresh_points_list()
            ts = data.get('timestamp', 'unknown')[:10]
            self.calib_status.config(
                text=f"✓ Loaded  ({len(self.calib_points)} pts, {ts})",
                foreground='green')
            if len(self.calib_points) >= 4:
                self.compute_btn.config(state='normal')
            self._log(f"Calibration loaded from file ({ts}).")
        except Exception as e:
            self._log(f"Failed to load calibration: {e}")

    def _load_calibration_dialog(self):
        """Manual reload (Load button)."""
        if os.path.exists(CALIB_FILE):
            self._load_calibration()
        else:
            messagebox.showwarning("Not found",
                                   f"No calibration file found at:\n{CALIB_FILE}")

    # ═══════════════════════════════════════════════════════════════ helpers ══
    def _on_mode_change(self):
        self._log(f"Mode → {self.mode.get()}")

    def _log(self, msg):
        def _do():
            self.log.configure(state='normal')
            self.log.insert(tk.END, msg + "\n")
            self.log.see(tk.END)
            self.log.configure(state='disabled')
        self.root.after(0, _do)

    # ══════════════════════════════════════════ auto-calibration routines ══════

    def _ac_toggle_robot(self):
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None
            self.ac_robot_btn.config(text="Connect Robot")
            self.ac_robot_lbl.config(text="⚫ Not connected", foreground='red')
            self.ac_start_btn.config(state='disabled')
            self._log("Robot disconnected.")
            return

        port = self.ac_port_var.get().strip()
        self.ac_robot_lbl.config(text="🟡 Connecting…", foreground='orange')
        self.ac_robot_btn.config(state='disabled')

        def do():
            try:
                from raspberry_pi_controller import RobotArmController
                robot = RobotArmController(port=port)
                if robot.connect():
                    self.robot = robot
                    def connected():
                        self.ac_robot_lbl.config(text="🟢 Connected", foreground='green')
                        self.ac_robot_btn.config(text="Disconnect Robot", state='normal')
                        self._ac_update_start_state()
                        self._log(f"Robot connected on {port}.")
                    self.root.after(0, connected)
                else:
                    def failed():
                        self.ac_robot_lbl.config(text="⚫ Failed", foreground='red')
                        self.ac_robot_btn.config(text="Connect Robot", state='normal')
                        self._log(f"✗ Robot connection failed on {port}.")
                    self.root.after(0, failed)
            except Exception as e:
                err = str(e)
                def error():
                    self.ac_robot_lbl.config(text="⚫ Error", foreground='red')
                    self.ac_robot_btn.config(text="Connect Robot", state='normal')
                    self._log(f"✗ Robot error: {err}")
                self.root.after(0, error)

        threading.Thread(target=do, daemon=True).start()

    def _ac_update_start_state(self):
        ready = (self.robot is not None
                 and self.tip_hsv is not None
                 and not self.ac_running)
        self.ac_start_btn.config(state='normal' if ready else 'disabled')

    def _ac_enter_pick_mode(self):
        if self.latest_frame is None:
            self._log("Connect the camera stream first, then click the tip colour.")
            return
        self.ac_pick_mode = True
        self.ac_color_btn.config(text="▶ Click on gripper tip in image…")
        self._log("Colour-pick mode: click the coloured gripper tip in the camera feed.")

    def _ac_set_color_from_frame(self, fx, fy):
        """Sample HSV from a patch around (fx, fy) and store as tip colour range."""
        with self.frame_lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None
        self.ac_pick_mode = False
        self.ac_color_btn.config(text="Click tip in frame")
        if frame is None:
            self._log("No frame available.")
            return

        fh, fw = frame.shape[:2]
        px = int(np.clip(fx, 0, fw - 1))
        py = int(np.clip(fy, 0, fh - 1))

        r = 3
        patch = frame[max(0, py - r):py + r + 1, max(0, px - r):px + r + 1]
        if patch.size == 0:
            self._log("Clicked outside frame bounds.")
            return

        hsv_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        h_mean = float(cv2.mean(hsv_patch)[0])
        s_mean = float(cv2.mean(hsv_patch)[1])
        v_mean = float(cv2.mean(hsv_patch)[2])

        tol_h, tol_s, tol_v = 18, 70, 70
        self.tip_hsv = (
            max(0,   int(h_mean - tol_h)),
            min(179, int(h_mean + tol_h)),
            max(0,   int(s_mean - tol_s)),
            255,
            max(30,  int(v_mean - tol_v)),
            255,
        )

        pr = min(r, patch.shape[0] - 1)
        pc = min(r, patch.shape[1] - 1)
        b, g, rv = int(patch[pr, pc, 0]), int(patch[pr, pc, 1]), int(patch[pr, pc, 2])
        hex_col = f'#{rv:02x}{g:02x}{b:02x}'
        self.ac_swatch.config(bg=hex_col)
        self.ac_color_lbl.config(
            text=f" H={int(h_mean)}° S={int(s_mean)} V={int(v_mean)}"
                 f"  (±{tol_h}° / ±{tol_s} / ±{tol_v})",
            foreground='black')
        self._log(f"Tip colour set: H={int(h_mean)}°±{tol_h}, "
                  f"S≥{max(0,int(s_mean-tol_s))}, V≥{max(30,int(v_mean-tol_v))}")
        self._ac_update_start_state()

    def _ac_start(self):
        if self.ac_running:
            self.ac_running = False
            self.ac_start_btn.config(text="▶  Start Auto-Calibrate")
            self._log("Auto-calibration cancelled.")
            return

        if self.robot is None:
            messagebox.showwarning("Robot not connected", "Connect the robot first.")
            return
        if self.tip_hsv is None:
            messagebox.showwarning("No tip colour",
                                   "Click 'Click tip in frame' to sample the gripper tip colour.")
            return
        if not self.streaming or self.latest_frame is None:
            messagebox.showwarning("No camera", "Connect the camera stream first.")
            return

        rows = self.ac_rows_var.get()
        cols = self.ac_cols_var.get()
        xs   = np.linspace(self.ac_xmin_var.get(), self.ac_xmax_var.get(), cols)
        ys   = np.linspace(self.ac_ymin_var.get(), self.ac_ymax_var.get(), rows)
        grid = [(float(x), float(y)) for y in ys for x in xs]

        self.ac_running = True
        self.ac_progress.config(value=0)
        self.ac_start_btn.config(text="■  Stop")
        self.ac_status_lbl.config(text=f"Running …  0/{len(grid)}", foreground='blue')
        threading.Thread(target=self._ac_run, args=(grid,), daemon=True).start()

    def _ac_run(self, grid_points):
        import time
        n         = len(grid_points)
        rz        = self.ac_z_var.get()
        settle    = self.ac_settle_var.get()
        collected = 0

        self.calib_points.clear()
        self.root.after(0, self._refresh_points_list)

        for i, (rx, ry) in enumerate(grid_points):
            if not self.ac_running:
                break

            status = f"Point {i+1}/{n}: moving to ({rx:.1f}, {ry:.1f}) cm…"
            self.root.after(0, lambda s=status: self.ac_status_lbl.config(
                text=s, foreground='blue'))
            self._log(f"→ {status}")

            if not self._move_arm_to(rx, ry, rz):
                self._log("  ✗ IK failed or unreachable — skipping")
            else:
                time.sleep(settle)
                with self.frame_lock:
                    frame = (self.latest_frame.copy()
                             if self.latest_frame is not None else None)
                if frame is None:
                    self._log("  ✗ No camera frame — skipping")
                else:
                    tip = self._detect_tip(frame)
                    if tip is None:
                        self._log("  ✗ Tip not detected — skipping "
                                  "(check colour or add lighting)")
                    else:
                        px, py = tip
                        self.calib_points.append(
                            {"pixel": [float(px), float(py)], "robot": [rx, ry]})
                        collected += 1
                        self._log(f"  ✓ pixel ({px},{py}) → robot ({rx:.1f},{ry:.1f})")
                        self.root.after(0, self._refresh_points_list)

            pct = int((i + 1) / n * 100)
            self.root.after(0, lambda p=pct: self.ac_progress.config(value=p))

        self.ac_running = False
        self.root.after(0, lambda: self.ac_start_btn.config(
            text="▶  Start Auto-Calibrate"))

        if collected >= 4:
            self._log(f"Collected {collected}/{n} points — computing homography…")
            self.root.after(0, self._compute_calibration)
            self.root.after(0, lambda: self.ac_status_lbl.config(
                text=f"✓ Done — {collected}/{n} pts", foreground='green'))
        else:
            self._log(f"✗ Only {collected}/{n} detected — need ≥ 4. "
                      "Recheck tip colour or grid range.")
            self.root.after(0, lambda: self.ac_status_lbl.config(
                text=f"✗ {collected}/{n} detected — recheck tip colour",
                foreground='red'))

        try:
            self.robot.home_position(smooth=True, display_progress=False)
        except Exception:
            pass

    def _move_arm_to(self, rx, ry, rz, steps=25, dt=0.07):
        """Move arm to (rx, ry, rz) via joint-space interpolation. Returns True if OK."""
        import time
        from inverse_kinematics_servo import calculate_ik
        try:
            current = self.robot.get_current_angles()
            twist   = current['t']
            guess   = {k: current[k] for k in ('b', 's', 'e', 'w')}
            target, ok, err = calculate_ik(rx, ry, rz, guess)
            if err > 3.0:
                return False
            j_cur = {k: current[k] for k in ('b', 's', 'e', 'w')}
            for step in range(1, steps + 1):
                t = step / steps
                cmd = {k: int(round(j_cur[k] + t * (target[k] - j_cur[k])))
                       for k in ('b', 's', 'e', 'w')}
                cmd['t'] = twist
                cmd['g'] = current['g']
                self.robot._stream_command(cmd)
                time.sleep(dt)
            with self.robot.feedback_lock:
                self.robot.current_angles.update(
                    {**target, 't': twist, 'g': current['g']})
            return True
        except Exception as e:
            self._log(f"  move error: {e}")
            return False

    def _detect_tip(self, frame):
        """Find the largest colour blob matching tip_hsv. Returns (px, py) or None."""
        if self.tip_hsv is None:
            return None
        h_min, h_max, s_min, s_max, v_min, v_max = self.tip_hsv
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([h_min, s_min, v_min], np.uint8),
                           np.array([h_max, s_max, v_max], np.uint8))
        # Handle red hue wraparound
        if h_min <= 10 or h_max >= 170:
            mask = cv2.bitwise_or(mask, cv2.inRange(
                hsv, np.array([0,   s_min, v_min], np.uint8),
                     np.array([10,  s_max, v_max], np.uint8)))
            mask = cv2.bitwise_or(mask, cv2.inRange(
                hsv, np.array([170, s_min, v_min], np.uint8),
                     np.array([179, s_max, v_max], np.uint8)))
        k    = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, k)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 50:
            return None
        M = cv2.moments(c)
        if M['m00'] == 0:
            return None
        return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

    def on_closing(self):
        self.streaming  = False
        self.ac_running = False
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = CameraCalibrationTool(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
