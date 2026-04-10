#!/usr/bin/env python3
"""
Click-to-Move: Click a point in the camera feed → robot arm goes there.

Requirements:
  - camera_calibration.json must exist (run camera_calibration.py first)
  - ESP32-CAM streaming
  - Robot connected via serial

Usage:
  python click_to_move.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import numpy as np
import cv2
from PIL import Image, ImageTk
import requests

from raspberry_pi_controller import RobotArmController
from camera_calibration import load_calibration, pixel_to_robot

DISPLAY_W = 640
DISPLAY_H = 480


class ClickToMove:

    def __init__(self, root):
        self.root = root
        self.root.title("Prime-Arm — Click to Move")
        self.root.geometry("960x620")
        self.root.resizable(True, True)

        # State
        self.robot      = None
        self.streaming  = False
        self.moving     = False
        self.homography = None

        self.latest_frame = None
        self.frame_lock   = threading.Lock()
        self._scale  = 1.0
        self._off_x  = 0
        self._off_y  = 0

        # Last clicked target (robot coords)
        self.target_xy    = None   # (rx, ry)
        self.target_pixel = None   # (px, py) in frame coords

        self._build_ui()
        self._load_calib()
        self._schedule_display()

    # ══════════════════════════════════════════════════════════════ UI ════════
    def _build_ui(self):
        self.root.columnconfigure(0, weight=3)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # ── Left: camera feed ─────────────────────────────────────────────────
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
        self.cam_btn = ttk.Button(cam_f, text="Connect Cam", command=self._toggle_stream)
        self.cam_btn.pack(side=tk.LEFT, padx=6)
        self.led_on = False
        self.led_btn = ttk.Button(cam_f, text="💡 LED On", command=self._toggle_led)
        self.led_btn.pack(side=tk.LEFT, padx=(0, 6))
        self.cam_status = ttk.Label(cam_f, text="⚫ No stream", foreground='red',
                                    font=('Arial', 9, 'bold'))
        self.cam_status.pack(side=tk.LEFT, padx=4)

        # Canvas
        canvas_f = ttk.LabelFrame(left,
            text="Camera Feed  —  click anywhere to move arm there", padding=4)
        canvas_f.grid(row=1, column=0, sticky='nsew')
        canvas_f.rowconfigure(0, weight=1)
        canvas_f.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_f, bg='#111',
                                width=DISPLAY_W, height=DISPLAY_H,
                                cursor='crosshair')
        self.canvas.grid(row=0, column=0, sticky='nsew')
        self.canvas.bind('<Button-1>', self._on_click)
        self.canvas.bind('<Motion>',   self._on_hover)

        self.coord_label = ttk.Label(left, text="Hover to preview robot coordinates",
                                     font=('Consolas', 9), foreground='gray')
        self.coord_label.grid(row=2, column=0, sticky='w', pady=(2, 0))

        # ── Right: robot controls ─────────────────────────────────────────────
        right = ttk.Frame(self.root, padding=8)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)

        # Robot connection
        rob_f = ttk.LabelFrame(right, text="Robot", padding=6)
        rob_f.grid(row=0, column=0, sticky='ew', pady=(0, 8))

        ttk.Label(rob_f, text="Port:").pack(side=tk.LEFT)
        import sys
        self.rob_port = tk.StringVar(
            value='COM4' if sys.platform.startswith('win') else '/dev/ttyUSB0')
        ttk.Entry(rob_f, textvariable=self.rob_port, width=8).pack(side=tk.LEFT, padx=4)
        self.rob_btn = ttk.Button(rob_f, text="Connect Robot", command=self._connect_robot)
        self.rob_btn.pack(side=tk.LEFT, padx=4)
        self.rob_status = ttk.Label(rob_f, text="⚫ Disconnected", foreground='red',
                                    font=('Arial', 9, 'bold'))
        self.rob_status.pack(side=tk.LEFT, padx=4)

        # Calibration status
        calib_f = ttk.LabelFrame(right, text="Calibration", padding=6)
        calib_f.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        self.calib_label = ttk.Label(calib_f, text="Loading…", foreground='gray',
                                     font=('Arial', 9), wraplength=200)
        self.calib_label.pack(anchor='w')

        # Move settings
        settings_f = ttk.LabelFrame(right, text="Move Settings", padding=6)
        settings_f.grid(row=2, column=0, sticky='ew', pady=(0, 8))
        settings_f.columnconfigure(1, weight=1)

        ttk.Label(settings_f, text="Height Z (cm):").grid(row=0, column=0, sticky='e', pady=2)
        self.z_var = tk.DoubleVar(value=5.0)
        ttk.Spinbox(settings_f, from_=0.5, to=20.0, increment=0.5,
                    textvariable=self.z_var, width=7).grid(row=0, column=1, sticky='w', padx=4)
        ttk.Label(settings_f, text="cm above table",
                  font=('Arial', 8), foreground='gray').grid(row=0, column=2, sticky='w')

        ttk.Label(settings_f, text="Speed dt (s):").grid(row=1, column=0, sticky='e', pady=2)
        self.dt_var = tk.DoubleVar(value=0.07)
        ttk.Spinbox(settings_f, from_=0.01, to=0.3, increment=0.01, format="%.2f",
                    textvariable=self.dt_var, width=7).grid(row=1, column=1, sticky='w', padx=4)

        ttk.Label(settings_f, text="Steps:").grid(row=2, column=0, sticky='e', pady=2)
        self.steps_var = tk.IntVar(value=30)
        ttk.Spinbox(settings_f, from_=5, to=100, increment=5,
                    textvariable=self.steps_var, width=7).grid(row=2, column=1, sticky='w', padx=4)

        # Target info
        tgt_f = ttk.LabelFrame(right, text="Last Click Target", padding=6)
        tgt_f.grid(row=3, column=0, sticky='ew', pady=(0, 8))
        self.target_label = ttk.Label(tgt_f, text="—", font=('Consolas', 9))
        self.target_label.pack(anchor='w')
        self.move_status = ttk.Label(tgt_f, text="", font=('Arial', 9, 'bold'),
                                     foreground='gray')
        self.move_status.pack(anchor='w', pady=(2, 0))

        # Quick actions
        act_f = ttk.LabelFrame(right, text="Actions", padding=6)
        act_f.grid(row=4, column=0, sticky='ew', pady=(0, 8))
        ttk.Button(act_f, text="🏠 Home",
                   command=self._go_home).pack(fill=tk.X, pady=2)
        ttk.Button(act_f, text="↩ Repeat Last Click",
                   command=self._repeat_move).pack(fill=tk.X, pady=2)

        # Log
        log_f = ttk.LabelFrame(right, text="Log", padding=4)
        log_f.grid(row=5, column=0, sticky='nsew', pady=(0, 4))
        right.rowconfigure(5, weight=1)
        log_f.columnconfigure(0, weight=1)
        log_f.rowconfigure(0, weight=1)
        self.log = scrolledtext.ScrolledText(log_f, height=8, font=('Consolas', 8),
                                              state='disabled', wrap=tk.WORD)
        self.log.grid(row=0, column=0, sticky='nsew')

    # ══════════════════════════════════════════════════════ calibration ════════
    def _load_calib(self):
        self.homography = load_calibration()
        if self.homography is not None:
            self.calib_label.config(
                text="✓ camera_calibration.json loaded", foreground='green')
            self._log("Calibration loaded.")
        else:
            self.calib_label.config(
                text="✗ No calibration found.\nRun camera_calibration.py first.",
                foreground='red')
            self._log("⚠ No calibration found — run camera_calibration.py first.")

    # ══════════════════════════════════════════════════════════ camera stream ══
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
            ip   = self.cam_ip.get().strip()
            port = self.cam_port.get().strip()
            url  = f"http://{ip}:{port}/"
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
            self._log(f"Camera stream connected: {url}")

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
                    jpg   = buf[s:e + 2]
                    buf   = buf[e + 2:]
                    frame = cv2.imdecode(
                        np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.frame_lock:
                            self.latest_frame = frame

        except requests.exceptions.ConnectionError:
            self._log(f"✗ Camera: connection refused at {url}")
        except Exception as e:
            self._log(f"✗ Camera stream error: {e}")
        finally:
            self.streaming = False
            self.root.after(0, lambda: [
                self.cam_btn.config(text="Connect Cam"),
                self.cam_status.config(text="⚫ Disconnected", foreground='red'),
            ])

    # ══════════════════════════════════════════════════════════════ display ════
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
            self.canvas.create_text(cw // 2, ch // 2,
                                    text="No camera feed",
                                    fill='gray', font=('Arial', 12))
            return

        # Draw overlays
        display = frame.copy()
        self._draw_overlay(display)

        fh, fw = display.shape[:2]
        scale  = min(cw / fw, ch / fh)
        nw, nh = int(fw * scale), int(fh * scale)
        off_x  = (cw - nw) // 2
        off_y  = (ch - nh) // 2

        self._scale = scale
        self._off_x = off_x
        self._off_y = off_y

        rgb = cv2.cvtColor(cv2.resize(display, (nw, nh)), cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, cw, ch, fill='#111', outline='')
        self.canvas.create_image(off_x, off_y, anchor='nw', image=img)
        self.canvas._img_ref = img

    def _draw_overlay(self, frame):
        # Draw the last clicked target
        if self.target_pixel is not None:
            px, py = int(self.target_pixel[0]), int(self.target_pixel[1])
            rx, ry = self.target_xy
            # Outer ring
            cv2.circle(frame, (px, py), 16, (0, 120, 255), 2)
            # Centre dot
            cv2.circle(frame, (px, py), 4, (0, 120, 255), -1)
            # Cross hair
            cv2.line(frame, (px - 22, py), (px + 22, py), (0, 120, 255), 1)
            cv2.line(frame, (px, py - 22), (px, py + 22), (0, 120, 255), 1)
            # Label
            cv2.putText(frame,
                        f"({rx:.1f}, {ry:.1f}) cm",
                        (px + 18, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 120, 255), 1, cv2.LINE_AA)

        # Draw robot workspace grid if calibrated
        if self.homography is not None:
            self._draw_grid(frame)

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
            col = (0, 200, 255) if x == 0 else (60, 60, 60)
            for j in range(len(pts) - 1):
                cv2.line(frame, pts[j], pts[j + 1], col, 1 + (x == 0), cv2.LINE_AA)

        for y in range(-extent, extent + 1, step):
            pts = [r2p(x, y) for x in dense]
            pts = [p for p in pts if p]
            col = (0, 200, 255) if y == 0 else (60, 60, 60)
            for j in range(len(pts) - 1):
                cv2.line(frame, pts[j], pts[j + 1], col, 1 + (y == 0), cv2.LINE_AA)

        # Origin marker
        o = r2p(0, 0)
        if o:
            cv2.circle(frame, o, 5, (0, 200, 255), -1)
            cv2.putText(frame, "origin", (o[0] + 7, o[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 255), 1)

    # ══════════════════════════════════════════════════════════ canvas events ══
    def _canvas_to_frame(self, cx, cy):
        return (cx - self._off_x) / self._scale, (cy - self._off_y) / self._scale

    def _on_hover(self, event):
        fx, fy = self._canvas_to_frame(event.x, event.y)
        if self.homography is not None:
            coords = pixel_to_robot(fx, fy, self.homography)
            if coords:
                self.coord_label.config(
                    text=f"→ Robot ({coords[0]:.1f}, {coords[1]:.1f}) cm"
                         f"   [pixel {fx:.0f}, {fy:.0f}]",
                    foreground='gray')
                return
        self.coord_label.config(text=f"Pixel ({fx:.0f}, {fy:.0f})", foreground='gray')

    def _on_click(self, event):
        if self.homography is None:
            messagebox.showwarning("No calibration",
                                   "Load calibration first (camera_calibration.py).")
            return
        if not self.robot:
            messagebox.showwarning("Not connected", "Connect the robot first.")
            return

        fx, fy = self._canvas_to_frame(event.x, event.y)
        coords = pixel_to_robot(fx, fy, self.homography)
        if coords is None:
            return

        rx, ry = coords
        self.target_pixel = (fx, fy)
        self.target_xy    = (rx, ry)
        self.target_label.config(text=f"X={rx:.1f} cm  Y={ry:.1f} cm")
        self._move_to(rx, ry)

    # ══════════════════════════════════════════════════════════ robot control ══
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
                    self._log(f"Robot connected. End-effector: "
                              f"({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) cm")
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

    def _move_to(self, rx, ry):
        if self.moving:
            self._log("⚠ Still moving — please wait.")
            return

        z    = self.z_var.get()
        dt   = self.dt_var.get()
        steps = self.steps_var.get()

        self._log(f"→ Moving to ({rx:.1f}, {ry:.1f}, {z:.1f}) cm …")
        self.move_status.config(text="Moving…", foreground='blue')

        def do():
            self.moving = True
            try:
                # Joint-space smooth move via send_command with smooth=True
                success = self.robot.move_to_position(rx, ry, z,
                                                      smooth=True,
                                                      display_progress=False)
                if success:
                    pos = self.robot.get_end_effector_position()
                    self._log(f"✓ Reached ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) cm")
                    self.root.after(0, lambda: self.move_status.config(
                        text="✓ Done", foreground='green'))
                else:
                    self._log("✗ IK failed — position may be unreachable.")
                    self.root.after(0, lambda: self.move_status.config(
                        text="✗ Unreachable", foreground='red'))
            except Exception as e:
                self._log(f"✗ Move error: {e}")
                self.root.after(0, lambda: self.move_status.config(
                    text=f"✗ Error", foreground='red'))
            finally:
                self.moving = False

        threading.Thread(target=do, daemon=True).start()

    def _repeat_move(self):
        if self.target_xy is None:
            return
        self._move_to(*self.target_xy)

    def _go_home(self):
        if not self.robot:
            messagebox.showwarning("Not connected", "Connect the robot first.")
            return
        if self.moving:
            return

        def do():
            self.moving = True
            self._log("🏠 Going home…")
            try:
                self.robot.home_position(smooth=True, display_progress=False)
                pos = self.robot.get_end_effector_position()
                self._log(f"✓ Home: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) cm")
                self.root.after(0, lambda: self.move_status.config(
                    text="Home", foreground='gray'))
            except Exception as e:
                self._log(f"✗ Home error: {e}")
            finally:
                self.moving = False

        threading.Thread(target=do, daemon=True).start()

    # ══════════════════════════════════════════════════════════════ helpers ════
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
    app = ClickToMove(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
