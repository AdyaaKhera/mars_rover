import tkinter as tk
from tkinter import ttk
import threading
import math
import time
import sys
import os
import serial
import serial.tools.list_ports
from collections import deque

cv2 = None
Image = None
ImageTk = None
pygame = None

GRID_SIZE = 8
CELL_SIZE = 34
PADDING = 20
MAP_CANVAS_WIDTH = PADDING * 2 + GRID_SIZE * CELL_SIZE
MAP_CANVAS_HEIGHT = PADDING * 2 + GRID_SIZE * CELL_SIZE
MINI_CAM_WIDTH = MAP_CANVAS_WIDTH
MINI_CAM_HEIGHT = int(MINI_CAM_WIDTH * 9 / 16)

BG_MAIN = "#0b1119"
BG_PANEL = "#10161f"
BG_CARD = "#171f2b"
FG_TITLE = "#dce8ff"
FG_TEXT = "#a9bddf"
GRID_UNKNOWN = "#1f2a38"
GRID_LINE = "#314359"
TREND_BG = "#0f1621"
TREND_GRID = "#243244"
TREND_LINE_DIST = "#ff7b72"
TREND_LINE_TEMP = "#f9cc5c"
TREND_LINE_LIGHT = "#52b6ff"
BARO_BAR_ACTIVE = "#73d6ff"
BARO_BAR_IDLE = "#1f2e42"
BARO_BAR_SPIKE = "#9ef2ff"
CAMERA_MIN_MEAN_AUTO = 8.0
RADAR_BG = "#0e1510"
RADAR_RING = "#1e6a3b"
RADAR_SWEEP = "#68f08a"
RADAR_SWEEP_CRIT = "#ff3b30"
RADAR_PING = "#3bd16f"
RADAR_PING_CRIT = "#ff4d4d"

TARGET_MIN_AREA_RATIO = 0.015
MOTION_ALERT_THRESHOLD = 50.0
EDGE_OBSTACLE_THRESHOLD = 0.095
GAMEPAD_AXIS_DEADZONE = 0.45
GAMEPAD_TRIGGER_ACTIVE_THRESHOLD = -0.70
UI_REFRESH_MS = 90
VAL_LOG_INTERVAL_MS = 1200
MOTION_TX_INTERVAL_MS = 60
PKT_PROCESS_MIN_MS = 55
VAL_PROCESS_MIN_MS = 75
CAMERA_RENDER_MIN_MS = 60

STATE_COLORS = {
    "BOOT": "#7f8c8d",
    "CAL": "#95a5a6",
    "SCAN": "#2c7da0",
    "HAZ": "#c63d2f",
    "SAMP": "#c18f00",
    "DARK": "#5b6c7a",
}

class MarsDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("Mars Rover Mission Control")
        self.root.configure(bg=BG_MAIN)
        self.root.geometry("1120x640")
        self.root.minsize(980, 560)

        self.style = ttk.Style(self.root)
        self.style.theme_use("clam")
        self.style.configure(
            "TCombobox",
            fieldbackground=BG_CARD,
            background=BG_CARD,
            foreground=FG_TITLE,
            bordercolor="#314359",
            lightcolor="#314359",
            darkcolor="#314359",
            arrowcolor="#9ec5ff",
            insertcolor=FG_TITLE,
            selectbackground="#223046",
            selectforeground="#e7f0ff"
        )
        self.style.map(
            "TCombobox",
            fieldbackground=[("readonly", BG_CARD)],
            foreground=[("readonly", "#e7f0ff")],
            selectbackground=[("readonly", "#223046")],
            selectforeground=[("readonly", "#e7f0ff")]
        )
        self.style.configure(
            "Mission.TButton",
            font=("Avenir", 10, "bold"),
            padding=(12, 6),
            background="#1e2b3c",
            foreground="#e8f0ff",
            bordercolor="#38506e",
            lightcolor="#38506e",
            darkcolor="#2a3d56"
        )
        self.style.map(
            "Mission.TButton",
            background=[("active", "#2a3d56"), ("pressed", "#162232")],
            foreground=[("active", "#ffffff"), ("pressed", "#d6e7ff")]
        )

        self.serial_conn = None
        self.reader_thread = None
        self.running = False

        self.grid_state = [["" for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self.latest = {
            "state": "BOOT",
            "sector": "A1",
            "distCm": "NA",
            "tempC": "NA",
            "light": "NA",
            "ultraPins": "A1/A3",
            "motorMode": "STOP",
            "motorPwm": "0",
            "motorEff": "S",
            "hazStop": "0",
            "baroKPa": "NA",
            "baroAltM": "NA",
            "baroOk": "0",
            "ms": "0",
        }

        self.logs = deque(maxlen=12)
        self.hist_dist = deque(maxlen=120)
        self.hist_temp = deque(maxlen=120)
        self.hist_light = deque(maxlen=120)
        self.hist_baro = deque(maxlen=120)
        self.sweep_angle = 0
        self.pulse_phase = 0
        self.last_ping_strength = 0.0
        self.hazard_flash = False
        self.fullscreen = False
        self.boot_ticks = 0
        self.boot_overlay = None
        self.boot_scanline = None
        self.camera_available = True
        self.camera_active = False
        self.camera_capture = None
        self.camera_source_index = None
        self.camera_source_name = None
        self.camera_sources = []
        self.camera_thread = None
        self.camera_frame = None
        self.camera_photo = None
        self.camera_canvas_photo = None
        self.camera_popout = None
        self.camera_popout_canvas = None
        self.camera_popout_photo = None
        self.camera_lock = threading.Lock()
        self.cv_enabled = True
        self.prev_gray = None
        self.cv_last_event = "INIT"
        self.frame_count = 0
        self.last_cv_boxes = []
        self.last_identifications = []

        self.qr_detector = None
        self.face_cascade = None
        self.hog = None

        self.title_var = tk.StringVar(value="Disconnected")
        self.state_value_var = tk.StringVar(value="BOOT")
        self.sector_value_var = tk.StringVar(value="A1")
        self.dist_value_var = tk.StringVar(value="NA cm")
        self.temp_value_var = tk.StringVar(value="NA C")
        self.light_value_var = tk.StringVar(value="NA")
        self.ultra_pins_var = tk.StringVar(value="A1/A3")
        self.motor_mode_var = tk.StringVar(value="STOP")
        self.motor_pwm_var = tk.StringVar(value="0")
        self.baro_kpa_var = tk.StringVar(value="NA kPa")
        self.baro_alt_var = tk.StringVar(value="NA m")
        self.ms_value_var = tk.StringVar(value="0")
        self.camera_status_var = tk.StringVar(value="Camera ready")
        self.camera_var = tk.StringVar(value="")
        self.cv_target_var = tk.StringVar(value="NONE")
        self.cv_terrain_var = tk.StringVar(value="PLAIN_TERRAIN")
        self.cv_motion_var = tk.StringVar(value="0.0")
        self.motion_score_var = tk.StringVar(value="0.0")
        self.cv_obstacle_var = tk.StringVar(value="CLEAR_PATH")
        self.cv_color_var = tk.StringVar(value="UNKNOWN")
        self.cv_color_target_var = tk.StringVar(value="NONE")
        self.motion_hazard_active = False
        self.last_motion_hazard_log = 0
        self.last_motion_tx_ms = 0
        self.last_motion_tx_score = None
        self.gamepad_enabled = False
        self.gamepad = None
        self.gamepad_name = ""
        self.gamepad_last_cmd = None
        self.gamepad_last_speed = None
        self.last_gamepad_scan = 0.0
        self.gamepad_status_var = tk.StringVar(value="Gamepad: disabled")
        self.last_val_log_ms = 0
        self.last_pkt_apply_ms = 0
        self.last_val_apply_ms = 0
        self.last_camera_render_ms = 0
        self.log_dirty = False
        self.grid_dirty = False

        self.build_ui()
        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.bind("<Escape>", self.exit_fullscreen)
        self.refresh_ports()
        self.camera_probe_on_start = os.environ.get("MARS_ENABLE_CAMERA", "0") == "1"
        if self.camera_probe_on_start:
            self.refresh_camera_sources()
        else:
            self.camera_sources = self.fallback_camera_sources()
            if hasattr(self, "camera_combo"):
                values = [f'{source["name"]} (#{source["index"]})' for source in self.camera_sources]
                self.camera_combo["values"] = values
                self.camera_var.set(values[0] if values else "")
            self.camera_status_var.set("Camera quick list loaded (set MARS_ENABLE_CAMERA=1 for full scan)")
        self.redraw_grid()
        self.create_boot_overlay()
        if self.camera_probe_on_start:
            self.camera_status_var.set("Camera idle")
        self.animate_ui()

    def build_ui(self):
        top = tk.Frame(self.root, bg=BG_PANEL, padx=12, pady=10)
        top.pack(fill="x", padx=10, pady=(10, 6))

        tk.Label(top, text="StarkHacks Mars Mission Control", font=("Avenir", 16, "bold"), bg=BG_PANEL, fg=FG_TITLE).pack(side="left")

        control = tk.Frame(top, bg=BG_PANEL)
        control.pack(side="right")

        tk.Label(control, text="Serial Port", font=("Avenir", 10, "bold"), bg=BG_PANEL, fg=FG_TEXT).pack(side="left", padx=(0, 6))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.pack(side="left", padx=4)

        tk.Label(control, text="Baud", font=("Avenir", 10, "bold"), bg=BG_PANEL, fg=FG_TEXT).pack(side="left", padx=(8, 4))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(control, textvariable=self.baud_var, width=8, state="readonly")
        self.baud_combo["values"] = ["115200", "9600", "57600", "38400"]
        self.baud_combo.pack(side="left", padx=4)
        self.baud_combo.set("115200")

        tk.Label(control, text="Camera", font=("Avenir", 10, "bold"), bg=BG_PANEL, fg=FG_TEXT).pack(side="left", padx=(8, 4))
        self.camera_combo = ttk.Combobox(control, textvariable=self.camera_var, width=20, state="readonly")
        self.camera_combo.pack(side="left", padx=4)
        self.camera_combo.bind("<<ComboboxSelected>>", self.on_camera_selected)

        self.connect_btn = ttk.Button(control, text="Connect", command=self.toggle_connection, style="Mission.TButton")
        self.connect_btn.pack(side="left", padx=4)

        self.refresh_btn = ttk.Button(control, text="Refresh", command=self.refresh_sources, style="Mission.TButton")
        self.refresh_btn.pack(side="left", padx=4)

        self.gamepad_btn = ttk.Button(control, text="Enable Gamepad", command=self.toggle_gamepad, style="Mission.TButton")
        self.gamepad_btn.pack(side="left", padx=4)

        self.gamepad_badge = tk.Label(control, textvariable=self.gamepad_status_var, font=("Avenir", 9, "bold"), bg="#1d2a3a", fg="#a9bddf", padx=8, pady=4)
        self.gamepad_badge.pack(side="left", padx=(4, 0))

        self.status_var = tk.StringVar(value="Disconnected")
        self.status_badge = tk.Label(control, textvariable=self.status_var, font=("Avenir", 10, "bold"), bg="#7f1d1d", fg="white", padx=10, pady=4)
        self.status_badge.pack(side="left", padx=(8, 0))

        main = tk.Frame(self.root, bg=BG_MAIN)
        main.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        top_row = tk.Frame(main, bg=BG_MAIN)
        top_row.pack(fill="both", expand=True)

        camera_col = tk.Frame(top_row, bg=BG_MAIN)
        camera_col.pack(side="left", fill="both", expand=True, padx=(0, 6))

        side_col = tk.Frame(top_row, bg=BG_MAIN, width=420)
        side_col.pack(side="left", fill="y")
        side_col.pack_propagate(False)

        camera_card = tk.Frame(camera_col, bg=BG_PANEL, padx=10, pady=8)
        camera_card.pack(fill="both", expand=True)
        tk.Label(camera_card, text="Camera View", font=("Avenir", 12, "bold"), bg=BG_PANEL, fg=FG_TITLE).pack(anchor="w")

        camera_header = tk.Frame(camera_card, bg=BG_PANEL)
        camera_header.pack(fill="x", pady=(4, 8))
        tk.Label(camera_header, textvariable=self.camera_status_var, bg=BG_PANEL, fg=FG_TEXT, font=("Avenir", 10, "bold")).pack(side="left")
        self.camera_expand_btn = ttk.Button(camera_header, text="Expand", command=self.toggle_camera_popout, style="Mission.TButton")
        self.camera_expand_btn.pack(side="right", padx=(6, 0))
        self.camera_btn = ttk.Button(camera_header, text="Start Camera", command=self.toggle_camera, style="Mission.TButton")
        self.camera_btn.pack(side="right")
        if not self.camera_available:
            self.camera_btn.config(state="disabled")
            self.cv_toggle_btn_state = "disabled"

        camera_actions = tk.Frame(camera_card, bg=BG_PANEL)
        camera_actions.pack(fill="x", pady=(0, 6))
        self.cv_toggle_btn = ttk.Button(camera_actions, text="CV ON", command=self.toggle_cv, style="Mission.TButton")
        self.cv_toggle_btn.pack(side="left")
        if not self.camera_available:
            self.cv_toggle_btn.config(state="disabled")

        self.camera_canvas = tk.Canvas(
            camera_card,
            width=700,
            height=430,
            bg="#081019",
            highlightthickness=1,
            highlightbackground="#2e3f56"
        )
        self.camera_canvas.pack(fill="both", expand=True)
        self.camera_canvas.bind("<Double-Button-1>", lambda _e: self.toggle_camera_popout())
        self.set_camera_placeholder()

        self.mission_card = tk.Frame(side_col, bg=BG_PANEL, padx=10, pady=8, highlightthickness=2, highlightbackground="#263349")
        self.mission_card.pack(fill="x", pady=(0, 6))
        tk.Label(self.mission_card, text="Mission Status", font=("Avenir", 12, "bold"), bg=BG_PANEL, fg=FG_TITLE).pack(anchor="w")
        self.state_badge = tk.Label(self.mission_card, textvariable=self.state_value_var, font=("Avenir", 10, "bold"), bg=STATE_COLORS["BOOT"], fg="white", padx=8, pady=3)
        self.state_badge.pack(anchor="w", pady=(6, 3))
        self.sector_label = tk.Label(self.mission_card, text="Sector A1", font=("Avenir", 10), bg=BG_PANEL, fg=FG_TEXT)
        self.sector_label.pack(anchor="w")

        metrics_card = tk.Frame(side_col, bg=BG_PANEL, padx=10, pady=8)
        metrics_card.pack(fill="x", pady=(0, 6))
        tk.Label(metrics_card, text="Telemetry", font=("Avenir", 12, "bold"), bg=BG_PANEL, fg=FG_TITLE).pack(anchor="w")
        grid = tk.Frame(metrics_card, bg=BG_PANEL)
        grid.pack(fill="x", pady=(6, 0))
        self.make_metric(grid, "Distance", self.dist_value_var, 0, 0, TREND_LINE_DIST)
        self.make_metric(grid, "Temp", self.temp_value_var, 0, 1, TREND_LINE_TEMP)
        self.make_metric(grid, "Light", self.light_value_var, 1, 0, TREND_LINE_LIGHT)
        self.make_metric(grid, "Pressure", self.baro_kpa_var, 1, 1, "#7ed8ff")
        self.make_metric(grid, "Motor", self.motor_mode_var, 2, 0, "#ffb86b")
        self.make_metric(grid, "PWM", self.motor_pwm_var, 2, 1, "#ffd27f")
        self.make_metric(grid, "Motion", self.motion_score_var, 3, 0, "#ff6b9d")
        self.make_metric(grid, "Altitude", self.baro_alt_var, 3, 1, "#9cd1ff")

        dynamics_card = tk.Frame(side_col, bg=BG_PANEL, padx=10, pady=8)
        dynamics_card.pack(fill="x", pady=(0, 6))
        tk.Label(dynamics_card, text="Live Telemetry", font=("Avenir", 12, "bold"), bg=BG_PANEL, fg=FG_TITLE).pack(anchor="w", pady=(0, 6))
        dynamics = tk.Frame(dynamics_card, bg=BG_PANEL)
        dynamics.pack(fill="x")
        self.trend_canvas = tk.Canvas(
            dynamics,
            width=250,
            height=120,
            bg=TREND_BG,
            highlightthickness=1,
            highlightbackground="#2e3f56"
        )
        self.trend_canvas.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.radar_canvas = tk.Canvas(
            dynamics,
            width=100,
            height=100,
            bg=RADAR_BG,
            highlightthickness=1,
            highlightbackground="#2e3f56"
        )
        self.radar_canvas.pack(side="left")
        self.baro_canvas = tk.Canvas(
            dynamics,
            width=120,
            height=100,
            bg="#0b1420",
            highlightthickness=1,
            highlightbackground="#2e3f56"
        )
        self.baro_canvas.pack(side="left", padx=(8, 0))

        motor_card = tk.Frame(side_col, bg=BG_PANEL, padx=10, pady=8)
        motor_card.pack(fill="x", pady=(0, 6))
        tk.Label(motor_card, text="Motor Control", font=("Avenir", 12, "bold"), bg=BG_PANEL, fg=FG_TITLE).pack(anchor="w", pady=(0, 6))
        
        # Direction buttons
        dir_frame = tk.Frame(motor_card, bg=BG_PANEL)
        dir_frame.pack(fill="x", pady=(0, 6))
        
        # Forward button (top center)
        fwd_btn = ttk.Button(dir_frame, text="↑ FWD", command=lambda: self.send_motor_cmd('F'), style="Mission.TButton")
        fwd_btn.pack(side="top", fill="x", pady=(0, 2))
        
        # Left, Stop, Right (middle row)
        mid_frame = tk.Frame(dir_frame, bg=BG_PANEL)
        mid_frame.pack(fill="x", pady=(0, 2))
        ttk.Button(mid_frame, text="← LEFT", command=lambda: self.send_motor_cmd('L'), style="Mission.TButton").pack(side="left", fill="x", expand=True, padx=(0, 2))
        ttk.Button(mid_frame, text="STOP", command=lambda: self.send_motor_cmd('S'), style="Mission.TButton").pack(side="left", fill="x", expand=True, padx=2)
        ttk.Button(mid_frame, text="RIGHT →", command=lambda: self.send_motor_cmd('R'), style="Mission.TButton").pack(side="left", fill="x", expand=True, padx=(2, 0))
        
        # Backward button (bottom center)
        ttk.Button(dir_frame, text="↓ BACK", command=lambda: self.send_motor_cmd('B'), style="Mission.TButton").pack(side="bottom", fill="x", pady=(2, 0))
        
        # Speed control
        speed_frame = tk.Frame(motor_card, bg=BG_PANEL)
        speed_frame.pack(fill="x")
        tk.Label(speed_frame, text="Speed:", bg=BG_PANEL, fg=FG_TEXT, font=("Avenir", 9)).pack(side="left", padx=(0, 6))
        self.motor_speed_var = tk.IntVar(value=8)
        speed_slider = tk.Scale(speed_frame, from_=0, to=10, orient="horizontal", variable=self.motor_speed_var, bg=BG_CARD, fg=FG_TEXT, highlightthickness=0, length=150)
        speed_slider.pack(side="left", fill="x", expand=True)
        self.motor_speed_label = tk.Label(speed_frame, text="80%", bg=BG_PANEL, fg=FG_TEXT, font=("Avenir", 9), width=4)
        self.motor_speed_label.pack(side="left", padx=(6, 0))
        speed_slider.config(command=self.update_speed_label)
        
        # Diagnostics button
        ttk.Button(motor_card, text="🔧 Motor Test", command=self.run_motor_diagnostics, style="Mission.TButton").pack(fill="x", pady=(6, 0))

        tk.Label(motor_card, text="Gamepad control is in the top toolbar.", bg=BG_PANEL, fg=FG_TEXT, font=("Avenir", 9, "bold")).pack(anchor="w", pady=(6, 0))

        log_card = tk.Frame(side_col, bg=BG_PANEL, padx=10, pady=8)
        log_card.pack(fill="both", expand=True)
        tk.Label(log_card, text="Recent Events", font=("Avenir", 12, "bold"), bg=BG_PANEL, fg=FG_TITLE).pack(anchor="w", pady=(0, 6))

        self.log_box = tk.Text(
            log_card,
            width=52,
            height=5,
            state="disabled",
            bg=BG_CARD,
            fg=FG_TEXT,
            relief="flat",
            font=("Menlo", 9),
            padx=6,
            pady=6
        )
        self.log_box.pack(fill="both", expand=True)

    def make_metric(self, parent, label, value_var, row, col, accent):
        card = tk.Frame(parent, bg=BG_CARD, padx=8, pady=6, highlightthickness=1, highlightbackground=accent)
        card.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)
        tk.Label(card, text=label, bg=BG_CARD, fg=accent, font=("Avenir", 8, "bold")).pack(anchor="w")
        tk.Label(card, textvariable=value_var, bg=BG_CARD, fg=accent, font=("Avenir", 10, "bold")).pack(anchor="w")
        parent.grid_columnconfigure(col, weight=1)

    def set_camera_placeholder(self, message=None):
        if not hasattr(self, "camera_canvas"):
            return
        if message is None:
            message = "Camera active" if self.camera_active else self.camera_status_var.get()
        self.camera_canvas.delete("all")
        canvas_w = max(1, int(self.camera_canvas.winfo_width()))
        canvas_h = max(1, int(self.camera_canvas.winfo_height()))
        self.camera_canvas.create_text(
            canvas_w // 2,
            canvas_h // 2,
            text=message,
            fill="#b9cee9",
            font=("Avenir", 16, "bold")
        )
        if hasattr(self, "camera_mini_canvas"):
            self.camera_mini_canvas.delete("all")
            self.camera_mini_canvas.create_text(
                MINI_CAM_WIDTH // 2,
                MINI_CAM_HEIGHT // 2,
                text=message,
                fill="#b9cee9",
                font=("Avenir", 10, "bold")
            )
        if self.camera_popout_canvas is not None:
            self.camera_popout_canvas.delete("all")
            self.camera_popout_canvas.create_text(
                480,
                270,
                text=message,
                fill="#b9cee9",
                font=("Avenir", 20, "bold")
            )
        self.camera_photo = None
        self.camera_canvas_photo = None
        self.camera_popout_photo = None

    def toggle_camera_popout(self):
        if self.camera_popout is not None and self.camera_popout.winfo_exists():
            self.close_camera_popout()
            return

        self.camera_popout = tk.Toplevel(self.root)
        self.camera_popout.title("Rover Camera - Expanded View")
        self.camera_popout.configure(bg=BG_MAIN)
        self.camera_popout.geometry("980x620")
        self.camera_popout.minsize(720, 460)

        header = tk.Frame(self.camera_popout, bg=BG_PANEL, padx=10, pady=8)
        header.pack(fill="x")
        tk.Label(header, text="Expanded Camera Feed", bg=BG_PANEL, fg=FG_TITLE, font=("Avenir", 14, "bold")).pack(side="left")

        self.camera_popout_canvas = tk.Canvas(
            self.camera_popout,
            width=960,
            height=540,
            bg="#081019",
            highlightthickness=1,
            highlightbackground="#2e3f56"
        )
        self.camera_popout_canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self.camera_expand_btn.config(text="Collapse")
        self.set_camera_placeholder("Camera active" if self.camera_active else "Camera stopped")

        self.camera_popout.protocol("WM_DELETE_WINDOW", self.close_camera_popout)

    def close_camera_popout(self):
        if self.camera_popout is not None and self.camera_popout.winfo_exists():
            self.camera_popout.destroy()
        self.camera_popout = None
        self.camera_popout_canvas = None
        self.camera_popout_photo = None
        if hasattr(self, "camera_expand_btn"):
            self.camera_expand_btn.config(text="Expand")

    def toggle_cv(self):
        self.cv_enabled = not self.cv_enabled
        self.cv_toggle_btn.config(text="CV ON" if self.cv_enabled else "CV OFF")
        if not self.cv_enabled:
            self.cv_target_var.set("OFF")
        self.append_log("CV enabled." if self.cv_enabled else "CV disabled.")

    def update_speed_label(self, val):
        speed = int(self.motor_speed_var.get())
        pct = min(100, speed * 10)
        self.motor_speed_label.config(text=f"{pct}%")

    def send_motor_cmd(self, cmd, quiet=False, source="UI"):
        """Send motor command to firmware: F=forward, B=backward, L=left, R=right, S=stop"""
        if not self.serial_conn:
            if not quiet:
                self.append_log("ERROR: No serial connection!")
            return
        
        try:
            # Send direction command
            self.serial_conn.write(cmd.encode())
            
            # Send speed command (0-9 or q for 100%).
            speed_level = int(self.motor_speed_var.get())
            if speed_level >= 10:
                speed_cmd = "q"
                pct = 100
            else:
                speed_cmd = str(max(0, min(9, speed_level)))
                pct = int(speed_cmd) * 10
            self.serial_conn.write(speed_cmd.encode())

            cmd_name = {"F": "FORWARD", "B": "BACKWARD", "L": "LEFT", "R": "RIGHT", "S": "STOP"}.get(cmd, cmd)
            if not quiet:
                prefix = "" if source == "UI" else f"[{source}] "
                self.append_log(f"{prefix}Motor: {cmd_name} @ {pct}%")
        except Exception as e:
            if not quiet:
                self.append_log(f"ERROR: Failed to send motor command: {e}")

    def toggle_gamepad(self):
        if self.gamepad_enabled:
            self.disable_gamepad()
        else:
            self.enable_gamepad()

    def enable_gamepad(self):
        global pygame
        if pygame is None:
            try:
                import pygame as pygame_mod
                pygame = pygame_mod
            except Exception as e:
                self.gamepad_status_var.set("Gamepad: pygame missing")
                self.append_log(f"Gamepad unavailable: install pygame ({e})")
                return

        try:
            pygame.init()
            pygame.joystick.init()
        except Exception as e:
            self.gamepad_status_var.set("Gamepad: init failed")
            self.append_log(f"Gamepad init failed: {e}")
            return

        if not self.scan_gamepad(force_log=True):
            self.gamepad_status_var.set("Gamepad: not found")
            return

        self.gamepad_enabled = True
        self.gamepad_btn.config(text="Disable Gamepad")
        self.gamepad_last_cmd = None
        self.gamepad_last_speed = None
        self.append_log("Gamepad control enabled.")

    def disable_gamepad(self):
        self.gamepad_enabled = False
        self.gamepad_btn.config(text="Enable Gamepad")
        self.gamepad_status_var.set("Gamepad: disabled")
        self.gamepad = None
        self.gamepad_name = ""
        self.gamepad_last_cmd = None
        self.gamepad_last_speed = None

        global pygame
        if pygame is not None:
            try:
                pygame.joystick.quit()
            except Exception:
                pass

        self.append_log("Gamepad control disabled.")

    def scan_gamepad(self, force_log=False):
        global pygame
        if pygame is None:
            return False

        try:
            pygame.joystick.quit()
            pygame.joystick.init()
            count = pygame.joystick.get_count()
        except Exception as e:
            self.gamepad_status_var.set("Gamepad: scan error")
            if force_log:
                self.append_log(f"Gamepad scan error: {e}")
            return False

        if count <= 0:
            self.gamepad = None
            self.gamepad_name = ""
            if force_log:
                self.append_log("No USB gamepad detected.")
            return False

        pick_index = 0
        picked_name = ""
        for idx in range(count):
            try:
                name = pygame.joystick.Joystick(idx).get_name() or "Unknown"
            except Exception:
                name = "Unknown"
            if "shanwan" in name.lower():
                pick_index = idx
                picked_name = name
                break
            if idx == 0:
                picked_name = name

        try:
            js = pygame.joystick.Joystick(pick_index)
            js.init()
            self.gamepad = js
            self.gamepad_name = picked_name or js.get_name() or f"Joystick {pick_index}"
            self.gamepad_status_var.set(f"Gamepad: {self.gamepad_name}")
            if force_log:
                self.append_log(f"Gamepad connected: {self.gamepad_name}")
            return True
        except Exception as e:
            self.gamepad = None
            self.gamepad_name = ""
            self.gamepad_status_var.set("Gamepad: open failed")
            if force_log:
                self.append_log(f"Gamepad open failed: {e}")
            return False

    def poll_gamepad(self):
        if not self.gamepad_enabled:
            return

        global pygame
        if pygame is None:
            return

        now = time.time()
        if (self.gamepad is None or not self.gamepad.get_init()) and (now - self.last_gamepad_scan) > 1.0:
            self.last_gamepad_scan = now
            self.scan_gamepad(force_log=False)
            return

        if self.gamepad is None:
            return

        try:
            pygame.event.pump()

            # D-pad preferred; fallback to left stick.
            hat_x, hat_y = (0, 0)
            if self.gamepad.get_numhats() > 0:
                hat_x, hat_y = self.gamepad.get_hat(0)

            axis_x = self.gamepad.get_axis(0) if self.gamepad.get_numaxes() > 0 else 0.0
            axis_y = self.gamepad.get_axis(1) if self.gamepad.get_numaxes() > 1 else 0.0
            deadzone = GAMEPAD_AXIS_DEADZONE

            # Optional speed by trigger axis. If trigger is idle, keep current UI speed.
            speed_level = int(self.motor_speed_var.get())
            if self.gamepad.get_numaxes() > 5:
                trigger_axis = self.gamepad.get_axis(5)
                if trigger_axis > GAMEPAD_TRIGGER_ACTIVE_THRESHOLD:
                    speed_level = int(max(0, min(10, round(((trigger_axis + 1.0) / 2.0) * 10.0))))

            desired = 'S'
            if hat_y > 0:
                desired = 'F'
            elif hat_y < 0:
                desired = 'B'
            elif hat_x < 0:
                desired = 'L'
            elif hat_x > 0:
                desired = 'R'
            elif abs(axis_y) >= abs(axis_x) and axis_y < -deadzone:
                desired = 'F'
            elif abs(axis_y) >= abs(axis_x) and axis_y > deadzone:
                desired = 'B'
            elif abs(axis_x) > abs(axis_y) and axis_x < -deadzone:
                desired = 'L'
            elif abs(axis_x) > abs(axis_y) and axis_x > deadzone:
                desired = 'R'

            # Any face button forces stop.
            for btn in range(min(4, self.gamepad.get_numbuttons())):
                if self.gamepad.get_button(btn):
                    desired = 'S'
                    break

            if speed_level != self.gamepad_last_speed:
                self.motor_speed_var.set(speed_level)
                self.update_speed_label(str(speed_level))
                self.gamepad_last_speed = speed_level

            if desired != self.gamepad_last_cmd:
                self.gamepad_last_cmd = desired
                self.send_motor_cmd(desired, quiet=True, source="GP")
                cmd_name = {"F": "FORWARD", "B": "BACKWARD", "L": "LEFT", "R": "RIGHT", "S": "STOP"}.get(desired, desired)
                self.append_log(f"[GP] {cmd_name} @ {min(100, speed_level * 10)}%")
        except Exception as e:
            self.gamepad_status_var.set("Gamepad: read error")
            self.append_log(f"Gamepad read error: {e}")
            self.gamepad = None

    def run_motor_diagnostics(self):
        """Send motor diagnostic request to firmware"""
        if not self.serial_conn:
            self.append_log("ERROR: No serial connection!")
            return
        
        try:
            self.serial_conn.write(b"D")
            self.append_log("Motor diagnostic test initiated (check serial output)...")
        except Exception as e:
            self.append_log(f"ERROR: Failed to send diagnostic: {e}")

    def draw_detect_box(self, frame, x, y, w, h, label, color):
        x2, y2 = x + w, y + h
        # Primary rectangle
        cv2.rectangle(frame, (x, y), (x2, y2), color, 2)
        # Corner accents for clearer HUD look
        seg = max(12, min(w, h) // 4)
        cv2.line(frame, (x, y), (x + seg, y), color, 3)
        cv2.line(frame, (x, y), (x, y + seg), color, 3)
        cv2.line(frame, (x2, y), (x2 - seg, y), color, 3)
        cv2.line(frame, (x2, y), (x2, y + seg), color, 3)
        cv2.line(frame, (x, y2), (x + seg, y2), color, 3)
        cv2.line(frame, (x, y2), (x, y2 - seg), color, 3)
        cv2.line(frame, (x2, y2), (x2 - seg, y2), color, 3)
        cv2.line(frame, (x2, y2), (x2, y2 - seg), color, 3)

        # Filled label bar for readability
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        bar_y1 = max(0, y - th - 10)
        bar_y2 = y
        bar_x2 = min(frame.shape[1] - 1, x + tw + 12)
        cv2.rectangle(frame, (x, bar_y1), (bar_x2, bar_y2), color, -1)
        cv2.putText(frame, label, (x + 6, bar_y2 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (10, 16, 26), 2)

    def toggle_camera(self):
        if self.camera_active:
            self.stop_camera()
        else:
            self.start_camera()

    def start_camera(self):
        if self.camera_active:
            return

        global cv2, Image, ImageTk
        if cv2 is None or ImageTk is None:
            try:
                import cv2 as cv2_mod
                from PIL import Image as PILImage, ImageTk as PILImageTk
                cv2 = cv2_mod
                Image = PILImage
                ImageTk = PILImageTk
            except Exception as e:
                self.camera_status_var.set(f"Camera libs unavailable: {e}")
                self.set_camera_placeholder("Camera libs unavailable")
                self.camera_btn.config(text="Start Camera")
                return

        backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
        candidate_indices = self.get_camera_candidate_indices()
        explicit_single_choice = len(candidate_indices) == 1
        self.camera_capture = None
        self.camera_source_index = None
        self.camera_source_name = None

        best_capture = None
        best_index = None
        best_name = None
        best_score = -1.0
        best_mean = -1.0

        for index in candidate_indices:
            capture = cv2.VideoCapture(index, backend)
            if capture and capture.isOpened():
                source_best_score = -1.0
                source_best_mean = -1.0
                for _ in range(6):
                    ok, frame = capture.read()
                    if not ok or frame is None or frame.size == 0:
                        continue
                    mean = float(frame.mean())
                    std = float(frame.std())
                    score = mean + 0.35 * std
                    if score > source_best_score:
                        source_best_score = score
                        source_best_mean = mean

                if source_best_score > best_score:
                    if best_capture:
                        best_capture.release()
                    best_capture = capture
                    best_index = index
                    best_name = self.describe_camera_source(index)
                    best_score = source_best_score
                    best_mean = source_best_mean
                else:
                    capture.release()
            elif capture:
                capture.release()

        if best_capture and (explicit_single_choice or best_mean >= CAMERA_MIN_MEAN_AUTO):
            self.camera_capture = best_capture
            self.camera_source_index = best_index
            self.camera_source_name = best_name
        elif best_capture:
            # Reject dark/near-empty feed when auto-selecting among multiple sources.
            best_capture.release()

        if not self.camera_capture or not self.camera_capture.isOpened():
            if self.camera_capture:
                self.camera_capture.release()
            self.camera_capture = None
            self.camera_active = False
            self.camera_status_var.set("Webcam not found")
            self.set_camera_placeholder("Webcam not found")
            self.camera_btn.config(text="Start Camera")
            return

        try:
            self.camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        except Exception:
            pass

        self.camera_active = True
        source_label = self.camera_source_name or f"#{self.camera_source_index}"
        self.camera_status_var.set(f"Camera live ({source_label})")
        self.camera_btn.config(text="Stop Camera")
        self.start_camera_thread()
        self.append_log(f"Camera started: {source_label}")

    def stop_camera(self):
        self.camera_active = False
        if self.camera_capture:
            try:
                self.camera_capture.release()
            except Exception:
                pass
        self.camera_capture = None
        self.camera_source_index = None
        self.camera_source_name = None
        self.camera_status_var.set("Camera stopped")
        self.camera_btn.config(text="Start Camera")
        self.set_camera_placeholder("Camera stopped")
        self.append_log("Camera stopped.")

    def get_camera_candidate_indices(self):
        if sys.platform == "darwin":
            default_candidates = [0, 1]
        else:
            default_candidates = [0, 1, 2, 3, 4, 5]
        sources = self.enumerate_camera_sources()
        if not sources:
            return default_candidates

        selected_index = self.get_selected_camera_index()
        if selected_index is not None:
            return [selected_index]

        preferred = []
        fallback = []
        seen = set()

        for source in sources:
            index = source["index"]
            name = source["name"].lower()
            if index in seen:
                continue
            seen.add(index)

            if "icspring" in name or "i cspring" in name or "c spring" in name:
                preferred.append(index)
            elif "continuity" in name or "iphone" in name or "phone" in name:
                continue
            else:
                fallback.append(index)

        if preferred:
            return preferred

        ordered = fallback + [idx for idx in default_candidates if idx not in seen]
        return ordered if ordered else default_candidates

    def refresh_camera_sources(self):
        self.camera_sources = self.enumerate_camera_sources()
        if not self.camera_sources:
            self.camera_sources = self.fallback_camera_sources()
        values = [f'{source["name"]} (#{source["index"]})' for source in self.camera_sources]
        self.camera_combo["values"] = values

        if not values:
            self.camera_var.set("")
            self.camera_status_var.set("No cameras detected")
            self.set_camera_placeholder("No cameras detected")
            return

        current = self.camera_var.get().strip()
        if current not in values:
            self.camera_var.set(self.find_preferred_camera_value(values))

    def find_preferred_camera_value(self, values):
        for value in values:
            if "icspring" in value.lower():
                return value
        return values[0]

    def get_selected_camera_index(self):
        selection = self.camera_var.get().strip()
        if not selection:
            return None

        for source in self.camera_sources:
            display = f'{source["name"]} (#{source["index"]})'
            if selection == display:
                return source["index"]

        if "(#" in selection and selection.endswith(")"):
            try:
                return int(selection.rsplit("(#", 1)[1].rstrip(")"))
            except Exception:
                return None
        return None

    def on_camera_selected(self, _event=None):
        selection = self.camera_var.get().strip()
        if selection:
            self.append_log(f"Camera selected: {selection}")

    def fallback_camera_sources(self):
        max_index = 3 if sys.platform == "darwin" else 2
        return [{"index": i, "name": f"Camera {i}", "id": str(i)} for i in range(max_index + 1)]

    def enumerate_camera_sources(self):
        # Avoid direct AVFoundation imports here; they have caused native crashes
        # on some macOS Python/OpenCV builds during app startup.
        global cv2
        if cv2 is None:
            try:
                import cv2 as cv2_mod
                cv2 = cv2_mod
            except Exception:
                return []

        max_index = 1 if sys.platform == "darwin" else 6
        try:
            max_index = int(os.environ.get("MARS_CAMERA_MAX_INDEX", str(max_index)))
        except Exception:
            max_index = 1 if sys.platform == "darwin" else 6
        max_index = max(0, min(max_index, 12))

        backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
        sources = []
        for index in range(max_index + 1):
            cap = None
            try:
                cap = cv2.VideoCapture(index, backend)
                if cap and cap.isOpened():
                    ok, frame = cap.read()
                    if ok and frame is not None and frame.size > 0:
                        sources.append({"index": index, "name": f"Camera {index}", "id": str(index)})
            except Exception:
                pass
            finally:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass

        return sources

    def describe_camera_source(self, index):
        for source in self.enumerate_camera_sources():
            if source["index"] == index:
                return source["name"]
        return None

    def refresh_camera_frame(self):
        if not self.camera_active or not self.camera_capture or ImageTk is None:
            return

        now_ms = int(time.time() * 1000)
        if now_ms - self.last_camera_render_ms < CAMERA_RENDER_MIN_MS:
            return
        self.last_camera_render_ms = now_ms

        frame = None
        with self.camera_lock:
            if self.camera_frame is not None:
                frame = self.camera_frame.copy()

        if frame is None:
            return

        try:
            display = frame
            if self.cv_enabled:
                display = self.analyze_frame(frame)

            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            canvas_w = max(1, int(self.camera_canvas.winfo_width()))
            canvas_h = max(1, int(self.camera_canvas.winfo_height()))
            image_full = Image.fromarray(rgb).resize((canvas_w, canvas_h))
            self.camera_photo = ImageTk.PhotoImage(image=image_full)
            self.camera_canvas.delete("all")
            self.camera_canvas_photo = self.camera_canvas.create_image(0, 0, anchor="nw", image=self.camera_photo)

            if hasattr(self, "camera_mini_canvas"):
                image_mini = Image.fromarray(rgb).resize((MINI_CAM_WIDTH, MINI_CAM_HEIGHT))
                self.camera_mini_photo = ImageTk.PhotoImage(image=image_mini)
                self.camera_mini_canvas.delete("all")
                self.camera_mini_canvas.create_image(0, 0, anchor="nw", image=self.camera_mini_photo)

            if self.camera_popout_canvas is not None:
                image_big = Image.fromarray(rgb).resize((960, 540))
                self.camera_popout_photo = ImageTk.PhotoImage(image=image_big)
                self.camera_popout_canvas.delete("all")
                self.camera_popout_canvas.create_image(0, 0, anchor="nw", image=self.camera_popout_photo)
        except Exception as e:
            self.camera_status_var.set(f"Camera render error: {e}")
            self.set_camera_placeholder("Camera render error")

    def analyze_frame(self, frame):
        self.frame_count += 1
        h, w = frame.shape[:2]
        out = frame.copy()

        # 1) Motion score using grayscale frame differencing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        motion_score = 0.0
        if self.prev_gray is not None:
            diff = cv2.absdiff(gray, self.prev_gray)
            motion_score = float(cv2.mean(diff)[0])
        self.prev_gray = gray

        # 2) Object identification pass (throttled every 3 frames)
        if self.frame_count % 3 == 0:
            detections = []
            labels = []

            # QR identification
            if self.qr_detector is not None:
                try:
                    qr_text, qr_points, _ = self.qr_detector.detectAndDecode(frame)
                    if qr_text and qr_points is not None and len(qr_points) > 0:
                        pts = qr_points.astype(int).reshape(-1, 2)
                        x, y, bw, bh = cv2.boundingRect(pts)
                        detections.append(("QR", x, y, bw, bh, (255, 200, 80)))
                        labels.append("QR")
                except Exception:
                    pass

            # Face identification
            if self.face_cascade is not None and not self.face_cascade.empty():
                faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(40, 40))
                for (x, y, bw, bh) in faces[:2]:
                    detections.append(("ALIEN_LIFEFORM", int(x), int(y), int(bw), int(bh), (120, 220, 255)))
                    labels.append("ALIEN_LIFEFORM")

            # Person identification (OpenCV HOG detector)
            if self.hog is not None:
                rects, weights = self.hog.detectMultiScale(frame, winStride=(8, 8), padding=(8, 8), scale=1.05)
                for i, (x, y, bw, bh) in enumerate(rects[:2]):
                    score = float(weights[i]) if i < len(weights) else 0.0
                    if score < 0.3:
                        continue
                    detections.append(("ALIEN_LIFEFORM", int(x), int(y), int(bw), int(bh), (90, 255, 140)))
                    labels.append("ALIEN_LIFEFORM")

            # Color object identification (red/blue/green largest blobs)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            color_specs = [
                (
                    "RED_OBJ",
                    [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
                    (80, 160, 255),
                ),
                (
                    "BLUE_OBJ",
                    [((95, 120, 60), (130, 255, 255))],
                    (255, 170, 90),
                ),
                (
                    "GREEN_OBJ",
                    [((35, 80, 60), (85, 255, 255))],
                    (120, 255, 120),
                ),
            ]

            min_blob_area = w * h * TARGET_MIN_AREA_RATIO
            for label, ranges, color in color_specs:
                mask = None
                for lo, hi in ranges:
                    m = cv2.inRange(hsv, lo, hi)
                    mask = m if mask is None else cv2.bitwise_or(mask, m)
                mask = cv2.GaussianBlur(mask, (5, 5), 0)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contours:
                    continue
                best = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(best)
                if area < min_blob_area:
                    continue
                x, y, bw, bh = cv2.boundingRect(best)
                detections.append((label, int(x), int(y), int(bw), int(bh), color))
                labels.append(label)

            self.last_cv_boxes = detections
            self.last_identifications = labels[:4]

        # Draw persisted detection boxes for visual stability
        for label, x, y, bw, bh, color in self.last_cv_boxes:
            self.draw_detect_box(out, x, y, bw, bh, label, color)

        target_name = ",".join(self.last_identifications[:2]) if self.last_identifications else "NONE"

        # 3) Obstacle cue from lower-half edge density
        lower = gray[h // 2 :, :]
        edges = cv2.Canny(lower, 80, 160)
        edge_density = float(cv2.countNonZero(edges)) / float(edges.size)
        obstacle = edge_density > EDGE_OBSTACLE_THRESHOLD
        terrain = "PLAIN_TERRAIN" if edge_density < 0.06 else "ROUGH_TERRAIN"

        # Dominant color estimate for Color tab
        hsv_all = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h_mean = float(hsv_all[:, :, 0].mean())
        s_mean = float(hsv_all[:, :, 1].mean())
        if s_mean < 35:
            dominant_color = "NEUTRAL"
        elif h_mean < 20 or h_mean > 160:
            dominant_color = "RED_ZONE"
        elif h_mean < 40:
            dominant_color = "YELLOW_ZONE"
        elif h_mean < 85:
            dominant_color = "GREEN_ZONE"
        elif h_mean < 135:
            dominant_color = "BLUE_ZONE"
        else:
            dominant_color = "PURPLE_ZONE"

        color_target = "NONE"
        for name in self.last_identifications:
            if name.endswith("_OBJ"):
                color_target = name
                break

        # Crosshair + HUD overlay
        cx, cy = w // 2, h // 2
        cv2.line(out, (cx - 20, cy), (cx + 20, cy), (130, 210, 255), 1)
        cv2.line(out, (cx, cy - 20), (cx, cy + 20), (130, 210, 255), 1)

        cv2.putText(out, f"MOTION {motion_score:.1f}", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 220, 120), 2)
        cv2.putText(out, f"EDGE {edge_density:.3f}", (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 220, 170), 2)

        if obstacle:
            cv2.putText(out, "OBSTACLE AHEAD", (w - 220, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 220, 120), 2)
        else:
            cv2.putText(out, terrain, (w - 230, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (130, 210, 255), 2)

        if motion_score > MOTION_ALERT_THRESHOLD:
            cv2.putText(out, "MOTION ALERT", (w - 180, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 200, 255), 2)

        self.cv_target_var.set(target_name)
        self.cv_terrain_var.set(terrain)
        self.cv_motion_var.set(f"{motion_score:.1f}")
        self.motion_score_var.set(f"{motion_score:.1f}")
        self.cv_obstacle_var.set("OBSTACLE_AHEAD" if obstacle else "CLEAR_PATH")
        self.cv_color_var.set(dominant_color)
        self.cv_color_target_var.set(color_target)

        # Motion-based hazard detection
        motion_hazard = motion_score > MOTION_ALERT_THRESHOLD
        if motion_hazard and not self.motion_hazard_active:
            self.motion_hazard_active = True
            self.append_log(f"MOTION HAZARD: score={motion_score:.1f} > {MOTION_ALERT_THRESHOLD}")
            self.send_motion_to_firmware(motion_score, "HAZARD")
        elif not motion_hazard and self.motion_hazard_active:
            self.motion_hazard_active = False
            self.append_log(f"MOTION CLEAR: score={motion_score:.1f}")
            self.send_motion_to_firmware(motion_score, "CLEAR")
        else:
            self.send_motion_to_firmware(motion_score)

        event = f"{target_name}|{terrain}|{'OBSTACLE' if obstacle else 'CLEAR'}|{dominant_color}|MOTION_{motion_score:.1f}"
        if event != self.cv_last_event:
            self.cv_last_event = event

        return out

    def send_motion_to_firmware(self, motion_score, state=None):
        if not self.serial_conn:
            return

        now_ms = int(time.time() * 1000)
        score_changed = self.last_motion_tx_score is None or abs(motion_score - self.last_motion_tx_score) >= 2.0
        if state is None:
            if (now_ms - self.last_motion_tx_ms) < MOTION_TX_INTERVAL_MS and not score_changed:
                return
            payload = f"MARS_MOTION|score={motion_score:.1f}\n"
        else:
            payload = f"MARS_MOTION|score={motion_score:.1f}|state={state}\n"

        try:
            self.serial_conn.write(payload.encode())
            self.last_motion_tx_ms = now_ms
            self.last_motion_tx_score = motion_score
        except Exception:
            pass

    def camera_loop(self):
        while self.camera_active and self.camera_capture:
            ok, frame = self.camera_capture.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            with self.camera_lock:
                self.camera_frame = frame
            time.sleep(0.01)

    def start_camera_thread(self):
        if self.camera_active and self.camera_capture and (self.camera_thread is None or not self.camera_thread.is_alive()):
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()

    def toggle_fullscreen(self, _event=None):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def exit_fullscreen(self, _event=None):
        self.fullscreen = False
        self.root.attributes("-fullscreen", False)

    def create_boot_overlay(self):
        self.boot_overlay = tk.Frame(self.root, bg="#05090f")
        self.boot_overlay.place(relx=0, rely=0, relwidth=1, relheight=1)

        title = tk.Label(
            self.boot_overlay,
            text="MARS MISSION CONTROL",
            font=("Avenir", 26, "bold"),
            bg="#05090f",
            fg="#dbe9ff"
        )
        title.pack(pady=(120, 8))

        self.boot_status = tk.Label(
            self.boot_overlay,
            text="Initializing telemetry matrix...",
            font=("Menlo", 12),
            bg="#05090f",
            fg="#86b7ff"
        )
        self.boot_status.pack(pady=(0, 18))

        self.boot_canvas = tk.Canvas(
            self.boot_overlay,
            width=760,
            height=100,
            bg="#0a1220",
            highlightthickness=1,
            highlightbackground="#2c3d57"
        )
        self.boot_canvas.pack()
        self.boot_scanline = self.boot_canvas.create_line(0, 50, 760, 50, fill="#55d2ff", width=2)
        self.boot_progress = self.boot_canvas.create_rectangle(20, 70, 20, 84, fill="#4adf86", outline="")

    def update_boot_overlay(self):
        if not self.boot_overlay:
            return

        self.boot_ticks += 1
        scan_y = 20 + (self.boot_ticks * 5) % 60
        self.boot_canvas.coords(self.boot_scanline, 0, scan_y, 760, scan_y)

        prog = min(1.0, self.boot_ticks / 45.0)
        x2 = 20 + int(720 * prog)
        self.boot_canvas.coords(self.boot_progress, 20, 70, x2, 84)

        if self.boot_ticks < 16:
            self.boot_status.config(text="Initializing telemetry matrix...")
        elif self.boot_ticks < 30:
            self.boot_status.config(text="Linking rover channel and map cache...")
        else:
            self.boot_status.config(text="Mission feed online")

        if self.boot_ticks > 55:
            self.boot_overlay.destroy()
            self.boot_overlay = None

    def refresh_ports(self):
        all_ports = [p.device for p in serial.tools.list_ports.comports()]
        preferred = [
            p for p in all_ports
            if ("usbmodem" in p.lower() or "usbserial" in p.lower() or "wchusbserial" in p.lower())
        ]
        if preferred:
            ports = preferred
        else:
            # Keep legacy ports visible, but de-prioritize obvious non-rover endpoints.
            ports = [
                p for p in all_ports
                if ("bluetooth" not in p.lower() and "debug" not in p.lower() and "wlan" not in p.lower())
            ]
            if not ports:
                ports = all_ports

        self.port_combo["values"] = ports

        current = self.port_var.get().strip()
        if ports and current not in ports:
            self.port_var.set(ports[0])
        elif ports and not current:
            self.port_var.set(ports[0])

    def refresh_sources(self):
        self.refresh_ports()
        self.refresh_camera_sources()

    def toggle_connection(self):
        if self.serial_conn:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            self.refresh_ports()
            port = self.port_var.get().strip()
        if not port:
            self.status_var.set("No port selected")
            return

        baud = 115200
        try:
            baud = int(self.baud_var.get().strip())
        except Exception:
            pass

        try:
            self.serial_conn = serial.Serial(port, baud, timeout=0.3)
        except serial.SerialException as e:
            msg = str(e)
            low = msg.lower()
            # On macOS, /dev/cu.* can be busy while /dev/tty.* is still usable.
            if "resource busy" in low and "/dev/cu." in port:
                alt_port = port.replace("/dev/cu.", "/dev/tty.")
                try:
                    self.serial_conn = serial.Serial(alt_port, baud, timeout=0.3)
                    port = alt_port
                except Exception:
                    self.serial_conn = None
            if "resource busy" in low or "permission" in low or "access is denied" in low:
                if not self.serial_conn:
                    self.status_var.set("Port busy: close Serial Monitor/Plotter, then retry")
                    self.append_log(f"PORT_BUSY {port}: {msg}")
            else:
                if not self.serial_conn:
                    self.status_var.set(f"Connect failed: {msg}")
                    self.append_log(f"CONNECT_ERR {port}: {msg}")
            if not self.serial_conn:
                self.serial_conn = None
                self.refresh_ports()
                return
        except Exception as e:
            self.status_var.set(f"Connect failed: {e}")
            self.append_log(f"CONNECT_ERR {port}: {e}")
            self.serial_conn = None
            return

        try:
            self.serial_conn.reset_input_buffer()
        except Exception:
            pass

        self.running = True
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

        self.title_var.set("Connected")
        self.status_var.set(f"Connected: {port} @ {baud}")
        self.status_badge.configure(bg="#166534")
        self.connect_btn.config(text="Disconnect")
        self.append_log("Connected.")

    def disconnect(self):
        self.running = False
        try:
            if self.serial_conn:
                self.serial_conn.close()
        except Exception:
            pass
        self.serial_conn = None
        self.status_var.set("Disconnected")
        self.status_badge.configure(bg="#7f1d1d")
        self.connect_btn.config(text="Connect")
        self.append_log("Disconnected.")

    def reader_loop(self):
        while self.running and self.serial_conn:
            try:
                line = self.serial_conn.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                self.root.after(0, self.handle_line, line)
            except Exception as e:
                self.root.after(0, self.append_log, f"Serial error: {e}")
                self.root.after(0, self.disconnect)
                break

    def handle_line(self, line):
        now_ms = int(time.time() * 1000)
        pkt_line = self.extract_packet_line(line)
        if pkt_line:
            if now_ms - self.last_pkt_apply_ms < PKT_PROCESS_MIN_MS:
                return
            data = self.parse_packet(pkt_line)
            if data:
                self.apply_packet(data)
                self.last_pkt_apply_ms = now_ms
        elif line.startswith("MARS_VAL"):
            if now_ms - self.last_val_apply_ms >= VAL_PROCESS_MIN_MS:
                data = self.parse_values(line)
                if data:
                    self.apply_values(data)
                    self.last_val_apply_ms = now_ms
            if now_ms - self.last_val_log_ms >= VAL_LOG_INTERVAL_MS:
                self.append_log(line)
                self.last_val_log_ms = now_ms
        elif line.startswith("MARS_CFG"):
            self.handle_config_line(line)
            self.append_log(line)
        elif line.startswith("MARS_BARO"):
            self.handle_baro_status_line(line)
            self.append_log(line)
        elif line.startswith("MARS_ALERT") or line.startswith("MARS_SUMMARY") or line.startswith("MARS_CAL"):
            self.append_log(line)
        elif line.startswith("MARS_ROVER_BOOT"):
            self.append_log("Rover booted.")
        else:
            pass

    def handle_baro_status_line(self, line):
        # Expected: MARS_BARO|status=OK or MARS_BARO|status=ERR
        status = ""
        parts = line.split("|")
        for p in parts[1:]:
            if p.startswith("status="):
                status = p.split("=", 1)[1].strip().upper()
                break
        if status == "ERR":
            self.baro_kpa_var.set("NA kPa")
            self.baro_alt_var.set("NA m")

    def handle_config_line(self, line):
        parts = line.split("|")
        cfg = {}
        for p in parts[1:]:
            if "=" in p:
                k, v = p.split("=", 1)
                cfg[k] = v

        trig = cfg.get("trigPin")
        echo = cfg.get("echoPin")
        if trig is not None and echo is not None:
            self.latest["ultraPins"] = f"{trig}/{echo}"
            self.ultra_pins_var.set(self.latest["ultraPins"])

    def extract_packet_line(self, line):
        # Tolerate garbage prefixes/noise due to serial resets or wrong baud transitions
        idx = line.find("MARS_PKT")
        if idx == -1:
            return None
        return line[idx:].strip()

    def parse_packet(self, line):
        # MARS_PKT|...|distCm=..|motorReq=..|motorEff=..|motorMode=..|motorPwm=..|hazStop=..
        parts = line.split("|")
        if not parts or parts[0] != "MARS_PKT":
            return None

        out = {}
        for p in parts[1:]:
            if "=" in p:
                k, v = p.split("=", 1)
                out[k] = v
        return out

    def parse_values(self, line):
        # MARS_VAL|ms=..|dist=..|temp=..|light=..|motor=..|baroKPa=..
        parts = line.split("|")
        if not parts or parts[0] != "MARS_VAL":
            return None

        out = {}
        for p in parts[1:]:
            if "=" in p:
                k, v = p.split("=", 1)
                out[k] = v
        return out

    def apply_values(self, data):
        dist = data.get("dist", self.latest["distCm"])
        temp = data.get("temp", self.latest["tempC"])
        light = data.get("light", self.latest["light"])
        motor = data.get("motor", self.latest["motorMode"])
        baro_kpa = data.get("baroKPa", self.latest["baroKPa"])
        ms = data.get("ms", self.latest["ms"])

        self.latest.update({
            "distCm": dist,
            "tempC": temp,
            "light": light,
            "motorMode": motor,
            "baroKPa": baro_kpa,
            "ms": ms,
        })

        self.dist_value_var.set(f"{dist} cm")
        self.temp_value_var.set(f"{temp} C")
        self.light_value_var.set(light)
        self.motor_mode_var.set(motor)
        self.baro_kpa_var.set(f"{baro_kpa} kPa")
        self.ms_value_var.set(ms)
        self.push_history(dist, temp, light, baro_kpa)

    def apply_packet(self, data):
        state = data.get("state", self.latest["state"])
        sector = data.get("sector", self.latest["sector"])
        light = data.get("light", self.latest["light"])
        tempC = data.get("tempC", self.latest["tempC"])
        dist = data.get("distCm", self.latest["distCm"])
        motor_mode = data.get("motorMode", self.latest["motorMode"])
        motor_pwm = data.get("motorPwm", self.latest["motorPwm"])
        baro_kpa = data.get("baroKPa", self.latest["baroKPa"])
        baro_alt = data.get("baroAltM", self.latest["baroAltM"])
        ms = data.get("ms", self.latest["ms"])

        self.latest.update({
            "state": state,
            "sector": sector,
            "light": light,
            "tempC": tempC,
            "distCm": dist,
            "motorMode": motor_mode,
            "motorPwm": motor_pwm,
            "motorEff": data.get("motorEff", self.latest["motorEff"]),
            "hazStop": data.get("hazStop", self.latest["hazStop"]),
            "baroKPa": baro_kpa,
            "baroAltM": baro_alt,
            "ms": ms
        })

        self.state_value_var.set(state)
        self.state_badge.configure(bg=STATE_COLORS.get(state, STATE_COLORS["BOOT"]))
        self.sector_label.config(text=f"Sector {sector}")
        self.dist_value_var.set(f"{dist} cm")
        self.temp_value_var.set(f"{tempC} C")
        self.light_value_var.set(light)
        self.ultra_pins_var.set(self.latest.get("ultraPins", "A1/A3"))
        self.motor_mode_var.set(motor_mode)
        self.motor_pwm_var.set(motor_pwm)
        self.baro_kpa_var.set(f"{baro_kpa} kPa")
        self.baro_alt_var.set(f"{baro_alt} m")
        self.ms_value_var.set(ms)

        self.push_history(dist, tempC, light, baro_kpa)
        if state == "HAZ":
            self.hazard_flash = not self.hazard_flash

        # update map cell
        pos = self.sector_to_xy(sector)
        if pos:
            x, y = pos
            if self.grid_state[y][x] != state:
                self.grid_state[y][x] = state
                self.grid_dirty = True

    def as_float_or_none(self, value):
        try:
            v = float(value)
            return v
        except Exception:
            return None

    def push_history(self, dist, temp, light, baro_kpa):
        d = self.as_float_or_none(dist)
        t = self.as_float_or_none(temp)
        l = self.as_float_or_none(light)
        b = self.as_float_or_none(baro_kpa)

        if d is None:
            d = self.hist_dist[-1] if self.hist_dist else 0.0
        if t is None:
            t = self.hist_temp[-1] if self.hist_temp else 0.0
        if l is None:
            l = self.hist_light[-1] if self.hist_light else 0.0
        if b is None:
            b = self.hist_baro[-1] if self.hist_baro else 0.0

        self.hist_dist.append(d)
        self.hist_temp.append(t)
        self.hist_light.append(l)
        self.hist_baro.append(b)

        if d > 0:
            self.last_ping_strength = max(0.05, min(1.0, 1.0 - (d / 120.0)))

    def draw_trends(self):
        c = self.trend_canvas
        c.delete("all")
        w = int(c["width"])
        h = int(c["height"])
        pad = 12

        # Grid
        for i in range(6):
            y = pad + i * (h - 2 * pad) / 5
            c.create_line(pad, y, w - pad, y, fill=TREND_GRID)
        for i in range(9):
            x = pad + i * (w - 2 * pad) / 8
            c.create_line(x, pad, x, h - pad, fill=TREND_GRID)

        c.create_text(pad + 4, 6, anchor="nw", text="LIVE SENSOR STREAM", fill="#a7c4eb", font=("Avenir", 9, "bold"))

        series = [
            (list(self.hist_dist), TREND_LINE_DIST, "Dist"),
            (list(self.hist_temp), TREND_LINE_TEMP, "Temp"),
            (list(self.hist_light), TREND_LINE_LIGHT, "Light"),
        ]

        # Dynamic scaling per stream for visual motion
        for values, color, _ in series:
            if len(values) < 2:
                continue
            vmin = min(values)
            vmax = max(values)
            if abs(vmax - vmin) < 0.001:
                vmax = vmin + 1.0

            points = []
            denom = max(1, len(values) - 1)
            for i, v in enumerate(values):
                x = pad + i * (w - 2 * pad) / denom
                norm = (v - vmin) / (vmax - vmin)
                y = h - pad - norm * (h - 2 * pad)
                points.extend((x, y))

            if len(points) >= 4:
                c.create_line(points, fill=color, width=2, smooth=True)

    def draw_radar(self):
        c = self.radar_canvas
        c.delete("all")
        w = int(c["width"])
        h = int(c["height"])
        cx = w // 2
        cy = h // 2
        r = min(w, h) // 2 - 10

        # Radar rings
        for ratio in (1.0, 0.75, 0.5, 0.25):
            rr = int(r * ratio)
            c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, outline=RADAR_RING)

        c.create_line(cx - r, cy, cx + r, cy, fill=RADAR_RING)
        c.create_line(cx, cy - r, cx, cy + r, fill=RADAR_RING)

        is_hazard = self.is_hazard_condition()
        is_critical = self.is_critical_distance()
        blink_red = is_hazard and ((self.pulse_phase % 8) < 4)
        sweep_color = RADAR_SWEEP_CRIT if is_hazard else RADAR_SWEEP
        ping_color = RADAR_PING_CRIT if is_hazard else RADAR_PING

        # Sweep arm
        angle_rad = math.radians(self.sweep_angle)
        x2 = cx + int(r * math.cos(angle_rad))
        y2 = cy - int(r * math.sin(angle_rad))
        c.create_line(cx, cy, x2, y2, fill=sweep_color, width=2)

        # Sweep glow wedge
        for i in range(1, 6):
            a = self.sweep_angle - i * 4
            ar = math.radians(a)
            xx = cx + int(r * math.cos(ar))
            yy = cy - int(r * math.sin(ar))
            if is_hazard:
                alpha_color = "#4b1414" if i > 2 else "#7f1d1d"
            else:
                alpha_color = "#12361f" if i > 2 else "#1c5c34"
            c.create_line(cx, cy, xx, yy, fill=alpha_color)

        # Distance ping pulse
        pulse = int((self.pulse_phase % 20) * 2 + 6)
        ping_r = int((r * 0.25) + self.last_ping_strength * (r * 0.6))
        c.create_oval(cx - ping_r, cy - ping_r, cx + ping_r, cy + ping_r, outline=ping_color)
        if is_hazard:
            fill_color = "#ff6b6b" if blink_red else "#b91c1c"
            c.create_oval(cx - pulse, cy - pulse, cx + pulse, cy + pulse, fill=fill_color, outline="")
        else:
            c.create_oval(cx - pulse, cy - pulse, cx + pulse, cy + pulse, fill=ping_color, outline="")

        # Center fill indicator: red while critical blink-red, otherwise green
        if is_hazard:
            center_fill = "#ff4d4d" if blink_red else "#8f1111"
        else:
            center_fill = RADAR_PING
        c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=center_fill, outline="")

        c.create_text(8, 8, anchor="nw", text="PROX RADAR", fill="#9fd9b8", font=("Avenir", 9, "bold"))

    def draw_barometer_bars(self):
        c = self.baro_canvas
        c.delete("all")
        w = int(c["width"])
        h = int(c["height"])
        pad = 8

        values = [v for v in self.hist_baro if v > 0]
        if len(values) < 2:
            c.create_text(w // 2, h // 2, text="BARO\nWAIT", fill="#8db9dc", font=("Avenir", 9, "bold"), justify="center")
            return

        recent = values[-24:]
        vmin = min(recent)
        vmax = max(recent)
        if abs(vmax - vmin) < 0.02:
            vmax = vmin + 0.02

        count = 16
        block_h = h - 2 * pad
        block_w = max(3, (w - 2 * pad) // count)
        samples = recent[-count:]
        if len(samples) < count:
            samples = [samples[0]] * (count - len(samples)) + samples

        for i, v in enumerate(samples):
            norm = max(0.0, min(1.0, (v - vmin) / (vmax - vmin)))
            bar_height = int(norm * block_h)
            x1 = pad + i * block_w
            x2 = x1 + block_w - 1
            y2 = h - pad
            y1 = y2 - bar_height
            color = BARO_BAR_SPIKE if i == count - 1 else (BARO_BAR_ACTIVE if norm > 0.33 else BARO_BAR_IDLE)
            c.create_rectangle(x1, y1, x2, y2, fill=color, outline="")

        current = samples[-1]
        previous = samples[-2]
        if current > previous + 0.002:
            trend = "UP"
            trend_color = "#7df5a6"
        elif current < previous - 0.002:
            trend = "DN"
            trend_color = "#ff9d9d"
        else:
            trend = "FL"
            trend_color = "#d7e7ff"

        c.create_text(6, 6, anchor="nw", text="BARO", fill="#89c5ff", font=("Avenir", 8, "bold"))
        c.create_text(w - 6, 6, anchor="ne", text=trend, fill=trend_color, font=("Avenir", 8, "bold"))
        c.create_text(w - 6, h - 6, anchor="se", text=f"{current:.2f}kPa", fill="#b7d8f5", font=("Avenir", 8, "bold"))

    def is_critical_distance(self):
        try:
            d = float(self.latest.get("distCm", "nan"))
            return d > 0 and d < 10.0
        except Exception:
            return False

    def is_hazard_condition(self):
        try:
            if self.latest.get("state", "") == "HAZ":
                return True
            if str(self.latest.get("hazStop", "0")) == "1":
                return True
            if self.motion_hazard_active:
                return True
            d = float(self.latest.get("distCm", "nan"))
            return d > 0 and d < 20.0
        except Exception:
            return False

    def animate_ui(self):
        self.sweep_angle = (self.sweep_angle + 8) % 360
        self.pulse_phase = (self.pulse_phase + 1) % 20

        self.poll_gamepad()

        self.update_boot_overlay()
        self.refresh_camera_frame()

        # Hazard pulse on status badge
        if self.latest.get("state") == "HAZ":
            self.status_badge.configure(bg="#d64534" if self.hazard_flash else "#8f2f23")
            self.mission_card.configure(highlightbackground="#ef4444" if self.hazard_flash else "#7f1d1d")
        elif self.serial_conn:
            self.status_badge.configure(bg="#166534")
            self.mission_card.configure(highlightbackground="#1f6c42")
        else:
            self.status_badge.configure(bg="#7f1d1d")
            self.mission_card.configure(highlightbackground="#263349")

        self.draw_trends()
        self.draw_radar()
        self.draw_barometer_bars()
        if self.grid_dirty:
            self.redraw_grid()
            self.grid_dirty = False
        self.render_logs()
        self.root.after(UI_REFRESH_MS, self.animate_ui)

    def sector_to_xy(self, sector):
        # A1..H8
        if len(sector) < 2:
            return None
        col_char = sector[0].upper()
        row_str = sector[1:]
        if not row_str.isdigit():
            return None

        x = ord(col_char) - ord("A")
        row = int(row_str)
        y = row - 1

        if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
            return (x, y)
        return None

    def redraw_grid(self):
        if not hasattr(self, "canvas"):
            return
        self.canvas.delete("all")

        # labels
        for i in range(GRID_SIZE):
            col = chr(ord("A") + i)
            self.canvas.create_text(PADDING + i * CELL_SIZE + CELL_SIZE // 2, 10, text=col, fill=FG_TEXT, font=("Avenir", 9, "bold"))
            self.canvas.create_text(10, PADDING + i * CELL_SIZE + CELL_SIZE // 2, text=str(i + 1), fill=FG_TEXT, font=("Avenir", 9, "bold"))

        # cells
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                x1 = PADDING + x * CELL_SIZE
                y1 = PADDING + y * CELL_SIZE
                x2 = x1 + CELL_SIZE
                y2 = y1 + CELL_SIZE

                st = self.grid_state[y][x]
                color = STATE_COLORS.get(st, GRID_UNKNOWN)
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline=GRID_LINE)

        # current rover position marker
        pos = self.sector_to_xy(self.latest["sector"])
        if pos:
            x, y = pos
            x1 = PADDING + x * CELL_SIZE
            y1 = PADDING + y * CELL_SIZE
            x2 = x1 + CELL_SIZE
            y2 = y1 + CELL_SIZE
            self.canvas.create_rectangle(x1 + 2, y1 + 2, x2 - 2, y2 - 2, outline="#111111", width=2)
            inset = max(6, int(CELL_SIZE * 0.28))
            self.canvas.create_oval(x1 + inset, y1 + inset, x2 - inset, y2 - inset, fill="#101010", outline="")

    def append_log(self, msg):
        self.logs.appendleft(msg)
        self.log_dirty = True

    def render_logs(self):
        if not self.log_dirty:
            return
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        for line in self.logs:
            self.log_box.insert("end", line + "\n")
        self.log_box.config(state="disabled")
        self.log_dirty = False

def run_dashboard_once():
    root = tk.Tk()
    app = MarsDashboard(root)
    started = time.time()
    last_close_attempt = 0.0

    def on_close():
        nonlocal last_close_attempt
        now = time.time()
        uptime = now - started

        # Guard against spurious early close events on some macOS GUI stacks.
        if uptime < 4.0 and (now - last_close_attempt) > 1.5:
            last_close_attempt = now
            print(f"MARS_DASH_GUARD|ignored_close|uptime={uptime:.2f}", flush=True)
            return

        app.stop_camera()
        app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    return time.time() - started


def main():
    print("MARS_DASH_START", flush=True)

    # If the window exits immediately due to transient GUI/backend issues,
    # attempt one automatic relaunch to keep startup reliable.
    uptime = run_dashboard_once()
    if uptime < 2.0:
        print(f"MARS_DASH_RESTART|reason=early_exit|uptime={uptime:.2f}", flush=True)
        uptime = run_dashboard_once()

    print("MARS_DASH_EXIT", flush=True)

if __name__ == "__main__":
    main()