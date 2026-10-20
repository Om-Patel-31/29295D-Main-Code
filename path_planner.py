#!/usr/bin/env python3
"""
===============================================================================
VEX V5 29295D PATH STUDIO — Visual Autonomous Path Generator
===============================================================================
A desktop GUI application inspired by path.jerryio.com for drawing and
configuring autonomous paths on a 144" x 144" VEX field, previewing
motion simulations, and directly writing production-ready C++ LADRC
motion commands (drive, turn, bdrive, sdrive) into src/autons.cpp.
===============================================================================
"""

import os
import sys
import math
import json
import re
import time
from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox
import customtkinter as ctk

# =============================================================================
# CONSTANTS & CONFIGURATION
# =============================================================================
FIELD_SIZE_IN = 144.0       # Standard VEX field size (12ft x 12ft)
CANVAS_SIZE_PX = 576        # Canvas pixel dimension (exactly 4.0 px per inch)
SCALE = CANVAS_SIZE_PX / FIELD_SIZE_IN # 4.0 pixels per inch
TILE_SIZE_IN = 24.0         # 24" x 24" foam tile size
ROBOT_WIDTH_IN = 14.5       # Robot physical width
ROBOT_LENGTH_IN = 14.5      # Robot physical length

# Colors (Dark Theme)
COLOR_BG_DARK = "#18191c"
COLOR_CANVAS_BG = "#1e2024"
COLOR_TILE_A = "#25282d"
COLOR_TILE_B = "#2a2d33"
COLOR_TILE_BORDER = "#353940"
COLOR_FIELD_WALL = "#4b515d"
COLOR_TAPE_WHITE = "#e5e7eb"
COLOR_RED_ALLIANCE = "#dc2626"
COLOR_BLUE_ALLIANCE = "#2563eb"
COLOR_PATH_LINE = "#38bdf8"
COLOR_PATH_SHADOW = "#0369a1"
COLOR_WAYPOINT_START = "#10b981"
COLOR_WAYPOINT_MID = "#0ea5e9"
COLOR_WAYPOINT_END = "#f43f5e"
COLOR_WAYPOINT_SEL = "#f59e0b"
COLOR_HEADING_LINE = "#fbbf24"
COLOR_ROBOT_BODY = "#64748b"
COLOR_ROBOT_FRONT = "#f97316"

# =============================================================================
# DATA STRUCTURES
# =============================================================================
@dataclass
class Waypoint:
    x: float               # X coordinate on field (inches, 0 = left, 144 = right)
    y: float               # Y coordinate on field (inches, 0 = bottom, 144 = top)
    heading: float         # Absolute heading (degrees, 0 = North/+Y, 90 = East/+X)
    motion_type: str = "bdrive"  # "bdrive" (Bezier), "drive" (Straight), "turn" (In-place)
    speed: float = 100.0   # Speed percentage (10% - 100%)
    reverse: bool = False  # If true, robot traverses segment in reverse (back leads)
    action: str = "none"   # "none", "clamp_claw", "open_claw", "lift_up", "lift_down", "sdrive", "wait"
    wait_ms: int = 0       # Optional pause duration in milliseconds

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        return cls(**data)


# =============================================================================
# COORDINATE CONVERSIONS & GEOMETRY HELPERS
# =============================================================================
def field_to_canvas(x_in: float, y_in: float) -> Tuple[float, float]:
    """Converts field coordinates (inches, origin bottom-left) to canvas pixels (origin top-left)."""
    cx = x_in * SCALE
    cy = (FIELD_SIZE_IN - y_in) * SCALE
    return cx, cy

def canvas_to_field(cx: float, cy: float) -> Tuple[float, float]:
    """Converts canvas pixel coordinates to field inches."""
    x = cx / SCALE
    y = FIELD_SIZE_IN - (cy / SCALE)
    return max(0.0, min(FIELD_SIZE_IN, x)), max(0.0, min(FIELD_SIZE_IN, y))

def normalize_angle(deg: float) -> float:
    """Normalizes an angle to [0, 360) degrees."""
    return deg % 360.0

def angle_diff(target_deg: float, current_deg: float) -> float:
    """Computes shortest signed angle difference in [-180, 180] degrees."""
    return ((target_deg - current_deg + 180.0) % 360.0) - 180.0

def sample_cubic_bezier(p0: Tuple[float, float], h0_deg: float,
                        p3: Tuple[float, float], h1_deg: float,
                        reverse: bool = False, n_samples: int = 35) -> List[Tuple[float, float, float]]:
    """
    Evaluates 2D cubic Bezier curve matching C++ BezierPath geometry.
    Returns list of (x_in, y_in, heading_deg) tuples.
    """
    x0, y0 = p0
    x3, y3 = p3
    chord = math.hypot(x3 - x0, y3 - y0)
    if chord < 1e-4:
        return [(x0, y0, h0_deg), (x3, y3, h1_deg)]

    d = 0.4 * chord
    sign = -1.0 if reverse else 1.0

    # Tangent vectors in field coordinates (heading 0 = +Y, 90 = +X)
    t0_x = sign * math.sin(math.radians(h0_deg))
    t0_y = sign * math.cos(math.radians(h0_deg))
    t1_x = sign * math.sin(math.radians(h1_deg))
    t1_y = sign * math.cos(math.radians(h1_deg))

    p1 = (x0 + d * t0_x, y0 + d * t0_y)
    p2 = (x3 - d * t1_x, y3 - d * t1_y)

    samples = []
    for i in range(n_samples + 1):
        t = i / n_samples
        u = 1.0 - t
        # Position
        px = (u**3)*p0[0] + 3*(u**2)*t*p1[0] + 3*u*(t**2)*p2[0] + (t**3)*p3[0]
        py = (u**3)*p0[1] + 3*(u**2)*t*p1[1] + 3*u*(t**2)*p2[1] + (t**3)*p3[1]
        # First derivative for heading
        dx = 3*(u**2)*(p1[0] - p0[0]) + 6*u*t*(p2[0] - p1[0]) + 3*(t**2)*(p3[0] - p2[0])
        dy = 3*(u**2)*(p1[1] - p0[1]) + 6*u*t*(p2[1] - p1[1]) + 3*(t**2)*(p3[1] - p2[1])
        if math.hypot(dx, dy) > 1e-6:
            heading = math.degrees(math.atan2(dx, dy)) % 360.0
            if reverse:
                heading = (heading + 180.0) % 360.0
        else:
            heading = h0_deg
        samples.append((px, py, heading))
    return samples


# =============================================================================
# C++ AUTONOMOUS CODE GENERATOR
# =============================================================================
def generate_cpp_code(waypoints: List[Waypoint], routine_name: str = "RS") -> str:
    """
    Converts list of field waypoints into formatted C++ motion commands
    compatible with the 29295D autonomous motion engine.
    """
    lines = []
    lines.append(f"// ============================================================================")
    lines.append(f"// Autonomous Routine: {routine_name}")
    lines.append(f"// Generated by 29295D Path Studio")
    lines.append(f"// Conventions: Distances in inches, Angles in degrees (+CW, -CCW)")
    lines.append(f"// ============================================================================")

    if len(waypoints) < 2:
        lines.append("sdrive();                                                      // Actively brake drivetrain")
        return "\n".join(lines)

    for i in range(1, len(waypoints)):
        p0 = waypoints[i - 1]
        p1 = waypoints[i]

        dx = p1.x - p0.x
        dy = p1.y - p0.y
        dist = math.hypot(dx, dy)
        h0 = p0.heading
        h1 = p1.heading
        speed = p1.speed
        speed_arg = f", {speed:.1f}" if abs(speed - 100.0) > 0.1 else ""
        m_type = p1.motion_type
        reverse = p1.reverse
        action = p1.action
        wait_ms = p1.wait_ms

        if m_type == "turn" or dist < 0.2:
            # In-place turn
            d_theta = angle_diff(h1, h0)
            if abs(d_theta) > 0.5:
                cmd = f"turn({d_theta:.1f}{speed_arg});"
                dir_label = "right" if d_theta > 0 else "left"
                comment = f"// Turn {dir_label} {abs(d_theta):.1f} deg to face {h1:.1f} deg"
                lines.append(f"{cmd:<36} {comment}")
        elif m_type == "drive":
            # Straight drive
            bearing = math.degrees(math.atan2(dx, dy)) % 360.0
            eff_dist = -dist if reverse else dist
            drive_heading = (bearing + 180.0) % 360.0 if reverse else bearing
            turn_in = angle_diff(drive_heading, h0)
            if abs(turn_in) > 1.0:
                cmd = f"turn({turn_in:.1f}{speed_arg});"
                dir_label = "right" if turn_in > 0 else "left"
                lines.append(f"{cmd:<36} // Align {dir_label} with straight path")
            
            cmd = f"drive({eff_dist:.1f}{speed_arg});"
            dir_label = "reverse" if reverse else "forward"
            lines.append(f"{cmd:<36} // Drive {dir_label} {abs(eff_dist):.1f} inches")

            turn_out = angle_diff(h1, drive_heading)
            if abs(turn_out) > 1.0:
                cmd = f"turn({turn_out:.1f}{speed_arg});"
                dir_label = "right" if turn_out > 0 else "left"
                lines.append(f"{cmd:<36} // Turn {dir_label} to target heading {h1:.1f} deg")
        else:
            # Bezier drive (bdrive)
            rad0 = math.radians(h0)
            fwd = dx * math.sin(rad0) + dy * math.cos(rad0)
            right = dx * math.cos(rad0) - dy * math.sin(rad0)
            end_h = angle_diff(h1, h0)
            if reverse:
                fwd = -abs(fwd)
            cmd = f"bdrive({fwd:.1f}, {right:.1f}, {end_h:.1f}{speed_arg});"
            side_str = "right" if right >= 0 else "left"
            lines.append(f"{cmd:<36} // Bezier: {fwd:.1f} in fwd, {abs(right):.1f} in {side_str}, turn {end_h:+.1f} deg")

        # Optional Actions
        if action == "clamp_claw":
            lines.append(f"{'claw.spin(forward, tuning.claw_close_power_pct, percent);':<36} // Clamp claw")
            lines.append(f"{f'wait({max(wait_ms, 200)}, msec);':<36} // Wait for clamp")
        elif action == "open_claw":
            lines.append(f"{'claw.spin(forward, tuning.claw_open_power_pct, percent);':<36} // Open claw")
            lines.append(f"{f'wait({max(wait_ms, 200)}, msec);':<36} // Wait for release")
        elif action == "lift_up":
            lines.append(f"{'lift_left.spin(forward, tuning.max_voltage, volt);':<36} // Raise cascade lift")
            lines.append(f"{'lift_right.spin(forward, tuning.max_voltage, volt);':<36} // Raise cascade lift")
        elif action == "lift_down":
            lines.append(f"{'lift_left.spin(reverse, tuning.max_voltage, volt);':<36} // Lower cascade lift")
            lines.append(f"{'lift_right.spin(reverse, tuning.max_voltage, volt);':<36} // Lower cascade lift")
        elif action == "sdrive":
            lines.append(f"{'sdrive();':<36} // Brake drivetrain")
        elif action == "wait" and wait_ms > 0:
            lines.append(f"{f'wait({wait_ms}, msec);':<36} // Pause delay")

    if not (lines and lines[-1].strip().startswith("sdrive();")):
        lines.append(f"{'sdrive();':<36} // Actively brake drivetrain at end of routine")
    return "\n".join(lines)


def write_code_to_autons_cpp(project_root: str, routine_name: str, code_body: str) -> Tuple[bool, str]:
    """
    Locates src/autons.cpp, backs it up, and replaces the target function body
    with the generated code.
    """
    autons_path = os.path.join(project_root, "src", "autons.cpp")
    if not os.path.exists(autons_path):
        return False, f"Could not find {autons_path}"

    try:
        with open(autons_path, "r", encoding="utf-8") as f:
            content = f.read()

        # Create backup copy
        bak_path = autons_path + ".bak"
        with open(bak_path, "w", encoding="utf-8") as f:
            f.write(content)

        # Match function definition
        pattern = rf"(void\s+{re.escape(routine_name)}\s*\(\s*\)\s*\{{)([\s\S]*?)(\n\}})"
        match = re.search(pattern, content)
        if not match:
            return False, f"Could not locate function 'void {routine_name}()' in src/autons.cpp"

        # Format body with 4-space indentation
        indented_lines = []
        for line in code_body.strip().splitlines():
            if line.strip():
                indented_lines.append("    " + line)
            else:
                indented_lines.append("")
        indented_body = "\n".join(indented_lines)

        new_content = content[:match.start(2)] + "\n" + indented_body + content[match.end(2):]

        with open(autons_path, "w", encoding="utf-8") as f:
            f.write(new_content)

        return True, f"Successfully updated void {routine_name}() in src/autons.cpp"
    except Exception as e:
        return False, f"Error writing to file: {str(e)}"


# =============================================================================
# GUI APPLICATION (CustomTkinter)
# =============================================================================
class PathStudioApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        self.title("VEX V5 29295D Path Studio — Autonomous Path Generator")
        self.geometry("1300x820")
        self.minsize(1180, 720)

        # Project directory
        self.project_root = os.path.abspath(os.path.dirname(__file__))

        # State
        self.waypoints: List[Waypoint] = []
        self.selected_idx: Optional[int] = None
        self.drag_mode: Optional[str] = None # "point", "heading"
        self.sim_active = False
        self.sim_t = 0.0

        # Load initial preset (RS)
        self.load_initial_preset("RS")

        # Build UI layout
        self._build_ui()
        self._draw_field()
        self._update_inspector()
        self._update_code_preview()

    def load_initial_preset(self, routine: str):
        preset_file = os.path.join(self.project_root, "paths", f"{routine}_default.json")
        if os.path.exists(preset_file):
            try:
                with open(preset_file, "r") as f:
                    data = json.load(f)
                    self.waypoints = [Waypoint.from_dict(w) for w in data["waypoints"]]
                    self.selected_idx = 0 if self.waypoints else None
                    return
            except Exception:
                pass
        # Fallback default starter points
        self.waypoints = [
            Waypoint(x=96.0, y=18.0, heading=0.0, motion_type="drive", speed=100.0),
            Waypoint(x=96.0, y=36.0, heading=0.0, motion_type="drive", speed=100.0),
            Waypoint(x=114.0, y=54.0, heading=90.0, motion_type="bdrive", speed=100.0)
        ]
        self.selected_idx = 0

    # -------------------------------------------------------------------------
    # UI Layout Construction
    # -------------------------------------------------------------------------
    def _build_ui(self):
        # Configure grid
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=0) # Left (Field)
        self.grid_columnconfigure(1, weight=1) # Right (Inspector & Code)

        # Header Frame
        header = ctk.CTkFrame(self, height=50, corner_radius=0, fg_color="#1e2024")
        header.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=0, pady=0)
        header.grid_columnconfigure(1, weight=1)

        title_lbl = ctk.CTkLabel(header, text="29295D PATH STUDIO", font=ctk.CTkFont(size=20, weight="bold"), text_color="#38bdf8")
        title_lbl.grid(row=0, column=0, padx=20, pady=10, sticky="w")

        sub_lbl = ctk.CTkLabel(header, text="Visual Autonomous Path Planner & C++ LADRC Code Generator", font=ctk.CTkFont(size=12), text_color="#94a3b8")
        sub_lbl.grid(row=0, column=1, padx=10, pady=10, sticky="w")

        status_lbl = ctk.CTkLabel(header, text="● Project Active", font=ctk.CTkFont(size=12, weight="bold"), text_color="#10b981")
        status_lbl.grid(row=0, column=2, padx=20, pady=10, sticky="e")

        # ----------------- LEFT PANEL: Field Canvas & Simulation -----------------
        left_panel = ctk.CTkFrame(self, corner_radius=12, fg_color="#18191c")
        left_panel.grid(row=1, column=0, padx=12, pady=12, sticky="nsew")

        # Coordinate bar
        self.coord_bar = ctk.CTkLabel(left_panel, text="Cursor: X: -- | Y: -- | Click to add waypoint", font=ctk.CTkFont(size=12, family="Consolas"), text_color="#94a3b8")
        self.coord_bar.pack(anchor="w", padx=12, pady=(10, 4))

        # Canvas Frame
        canvas_border = ctk.CTkFrame(left_panel, corner_radius=8, fg_color="#2b2d31", border_width=2, border_color="#3b4252")
        canvas_border.pack(padx=10, pady=4)

        self.canvas = tk.Canvas(canvas_border, width=CANVAS_SIZE_PX, height=CANVAS_SIZE_PX, bg=COLOR_CANVAS_BG, highlightthickness=0)
        self.canvas.pack(padx=2, pady=2)

        # Canvas event bindings
        self.canvas.bind("<ButtonPress-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.canvas.bind("<Motion>", self._on_canvas_hover)
        self.canvas.bind("<ButtonPress-3>", self._on_canvas_right_click)

        # Simulation & Stats Controls
        sim_frame = ctk.CTkFrame(left_panel, corner_radius=8, fg_color="#22252a")
        sim_frame.pack(fill="x", padx=10, pady=8)

        self.play_btn = ctk.CTkButton(sim_frame, text="▶ Play Sim", width=90, fg_color="#0284c7", hover_color="#0369a1", command=self._toggle_simulation)
        self.play_btn.pack(side="left", padx=8, pady=8)

        self.sim_speed_opt = ctk.CTkOptionMenu(sim_frame, values=["0.5x", "1.0x", "2.0x", "4.0x"], width=80)
        self.sim_speed_opt.set("1.0x")
        self.sim_speed_opt.pack(side="left", padx=4, pady=8)

        self.stats_lbl = ctk.CTkLabel(sim_frame, text="Length: 0.0 in | Time: 0.0 s", font=ctk.CTkFont(size=12, family="Consolas"), text_color="#38bdf8")
        self.stats_lbl.pack(side="right", padx=12, pady=8)

        # ----------------- RIGHT PANEL: Inspector & Code Preview -----------------
        right_panel = ctk.CTkFrame(self, corner_radius=12, fg_color="#18191c")
        right_panel.grid(row=1, column=1, padx=(0, 12), pady=12, sticky="nsew")
        right_panel.grid_rowconfigure(2, weight=1)
        right_panel.grid_columnconfigure(0, weight=1)

        # Top Control Card: Auton Dropdown & Mirror
        top_card = ctk.CTkFrame(right_panel, corner_radius=8, fg_color="#22252a")
        top_card.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))

        auton_lbl = ctk.CTkLabel(top_card, text="Target Auton:", font=ctk.CTkFont(size=13, weight="bold"))
        auton_lbl.pack(side="left", padx=(12, 6), pady=8)

        self.auton_dropdown = ctk.CTkOptionMenu(top_card, values=["RS", "LS", "Skills"], width=90, command=self._on_auton_select)
        self.auton_dropdown.set("RS")
        self.auton_dropdown.pack(side="left", padx=4, pady=8)

        mirror_btn = ctk.CTkButton(top_card, text="⇄ Mirror (X-Flip)", width=120, fg_color="#475569", hover_color="#334155", command=self._mirror_path)
        mirror_btn.pack(side="left", padx=6, pady=8)

        clear_btn = ctk.CTkButton(top_card, text="Clear Path", width=90, fg_color="#dc2626", hover_color="#b91c1c", command=self._clear_path)
        clear_btn.pack(side="right", padx=12, pady=8)

        # Waypoint Inspector Card
        self.inspector_card = ctk.CTkFrame(right_panel, corner_radius=8, fg_color="#22252a")
        self.inspector_card.grid(row=1, column=0, sticky="ew", padx=10, pady=4)

        self._build_inspector(self.inspector_card)

        # Code Preview Card
        code_card = ctk.CTkFrame(right_panel, corner_radius=8, fg_color="#22252a")
        code_card.grid(row=2, column=0, sticky="nsew", padx=10, pady=4)
        code_card.grid_rowconfigure(1, weight=1)
        code_card.grid_columnconfigure(0, weight=1)

        code_header = ctk.CTkFrame(code_card, fg_color="transparent")
        code_header.grid(row=0, column=0, sticky="ew", padx=10, pady=(8, 2))
        
        code_title = ctk.CTkLabel(code_header, text="Generated C++ Motion Code", font=ctk.CTkFont(size=13, weight="bold"), text_color="#38bdf8")
        code_title.pack(side="left")

        copy_btn = ctk.CTkButton(code_header, text="📋 Copy", width=70, height=24, fg_color="#475569", hover_color="#334155", command=self._copy_code)
        copy_btn.pack(side="right")

        self.code_text = ctk.CTkTextbox(code_card, font=ctk.CTkFont(size=12, family="Consolas"), fg_color="#18191c", text_color="#e2e8f0", wrap="none")
        self.code_text.grid(row=1, column=0, sticky="nsew", padx=10, pady=(2, 8))

        # Bottom Action Bar
        bottom_bar = ctk.CTkFrame(right_panel, corner_radius=8, fg_color="#22252a")
        bottom_bar.grid(row=3, column=0, sticky="ew", padx=10, pady=(4, 10))

        save_btn = ctk.CTkButton(bottom_bar, text="💾 Save JSON", width=100, fg_color="#334155", hover_color="#1e293b", command=self._save_path_dialog)
        save_btn.pack(side="left", padx=8, pady=10)

        load_btn = ctk.CTkButton(bottom_bar, text="📂 Load JSON", width=100, fg_color="#334155", hover_color="#1e293b", command=self._load_path_dialog)
        load_btn.pack(side="left", padx=4, pady=10)

        # BIG WRITE BUTTON
        self.write_btn = ctk.CTkButton(bottom_bar, text="⚡ WRITE TO AUTONS.CPP", font=ctk.CTkFont(size=14, weight="bold"),
                                       height=40, fg_color="#10b981", hover_color="#059669", command=self._write_to_codebase)
        self.write_btn.pack(side="right", padx=12, pady=10)

    # -------------------------------------------------------------------------
    # Waypoint Inspector Sub-Frame
    # -------------------------------------------------------------------------
    def _build_inspector(self, parent):
        row1 = ctk.CTkFrame(parent, fg_color="transparent")
        row1.pack(fill="x", padx=10, pady=(8, 4))

        self.wp_label = ctk.CTkLabel(row1, text="Waypoint: None", font=ctk.CTkFont(size=13, weight="bold"), text_color="#f59e0b")
        self.wp_label.pack(side="left")

        self.del_btn = ctk.CTkButton(row1, text="🗑 Delete", width=75, height=24, fg_color="#ef4444", hover_color="#dc2626", command=self._delete_selected)
        self.del_btn.pack(side="right")

        # Coordinates & Heading Inputs
        row2 = ctk.CTkFrame(parent, fg_color="transparent")
        row2.pack(fill="x", padx=10, pady=4)

        ctk.CTkLabel(row2, text="X (in):", width=40).pack(side="left", padx=(0, 2))
        self.entry_x = ctk.CTkEntry(row2, width=65)
        self.entry_x.pack(side="left", padx=(0, 10))
        self.entry_x.bind("<Return>", lambda e: self._on_coord_edited())

        ctk.CTkLabel(row2, text="Y (in):", width=40).pack(side="left", padx=(0, 2))
        self.entry_y = ctk.CTkEntry(row2, width=65)
        self.entry_y.pack(side="left", padx=(0, 10))
        self.entry_y.bind("<Return>", lambda e: self._on_coord_edited())

        ctk.CTkLabel(row2, text="Heading (°):", width=75).pack(side="left", padx=(0, 2))
        self.entry_h = ctk.CTkEntry(row2, width=65)
        self.entry_h.pack(side="left", padx=(0, 6))
        self.entry_h.bind("<Return>", lambda e: self._on_coord_edited())

        # Quick angle buttons
        for ang in [0, 90, 180, 270]:
            btn = ctk.CTkButton(row2, text=f"{ang}°", width=42, height=24, fg_color="#334155", hover_color="#1e293b",
                                command=lambda a=ang: self._set_heading_quick(a))
            btn.pack(side="left", padx=2)

        # Motion Type & Speed
        row3 = ctk.CTkFrame(parent, fg_color="transparent")
        row3.pack(fill="x", padx=10, pady=4)

        ctk.CTkLabel(row3, text="Motion:", width=50).pack(side="left", padx=(0, 4))
        self.motion_seg = ctk.CTkSegmentedButton(row3, values=["bdrive", "drive", "turn"], command=self._on_motion_type_changed)
        self.motion_seg.set("bdrive")
        self.motion_seg.pack(side="left", padx=(0, 15))

        self.reverse_chk = ctk.CTkCheckBox(row3, text="Reverse", command=self._on_reverse_toggled)
        self.reverse_chk.pack(side="left", padx=(0, 15))

        ctk.CTkLabel(row3, text="Speed:", width=45).pack(side="left", padx=(0, 2))
        self.speed_slider = ctk.CTkSlider(row3, from_=10, to=100, number_of_steps=18, width=110, command=self._on_speed_changed)
        self.speed_slider.set(100)
        self.speed_slider.pack(side="left", padx=4)

        self.speed_lbl = ctk.CTkLabel(row3, text="100%", width=40, font=ctk.CTkFont(family="Consolas"))
        self.speed_lbl.pack(side="left")

        # Action Selector
        row4 = ctk.CTkFrame(parent, fg_color="transparent")
        row4.pack(fill="x", padx=10, pady=(4, 8))

        ctk.CTkLabel(row4, text="Action:", width=50).pack(side="left", padx=(0, 4))
        self.action_opt = ctk.CTkOptionMenu(row4, values=["none", "clamp_claw", "open_claw", "lift_up", "lift_down", "sdrive", "wait"],
                                            width=120, command=self._on_action_changed)
        self.action_opt.set("none")
        self.action_opt.pack(side="left", padx=(0, 10))

        ctk.CTkLabel(row4, text="Wait (ms):", width=65).pack(side="left", padx=(0, 2))
        self.entry_wait = ctk.CTkEntry(row4, width=65)
        self.entry_wait.insert(0, "0")
        self.entry_wait.pack(side="left")
        self.entry_wait.bind("<Return>", lambda e: self._on_coord_edited())

    # -------------------------------------------------------------------------
    # Canvas Drawing & Field Rendering
    # -------------------------------------------------------------------------
    def _draw_field(self):
        self.canvas.delete("all")

        # 1. Draw 6x6 foam tiles
        tiles_count = 6
        tile_px = CANVAS_SIZE_PX / tiles_count
        for r in range(tiles_count):
            for c in range(tiles_count):
                x1 = c * tile_px
                y1 = r * tile_px
                x2 = x1 + tile_px
                y2 = y1 + tile_px
                color = COLOR_TILE_A if (r + c) % 2 == 0 else COLOR_TILE_B
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline=COLOR_TILE_BORDER, width=1)

        # 2. Draw Center Field Division Lines
        center_px = CANVAS_SIZE_PX / 2
        self.canvas.create_line(center_px, 0, center_px, CANVAS_SIZE_PX, fill=COLOR_TAPE_WHITE, width=2, dash=(6, 4))
        self.canvas.create_line(0, center_px, CANVAS_SIZE_PX, center_px, fill=COLOR_TAPE_WHITE, width=2, dash=(6, 4))

        # 3. Draw Center Structure (High Stakes Tower/Ladder 24" x 24")
        tw_in = 24.0
        t_half = (tw_in / 2.0) * SCALE
        self.canvas.create_rectangle(center_px - t_half, center_px - t_half, center_px + t_half, center_px + t_half,
                                     outline="#f59e0b", width=2, dash=(4, 2))

        # 4. Alliance Lines & Markers
        # Red side marker (right border)
        self.canvas.create_rectangle(CANVAS_SIZE_PX - 8, 0, CANVAS_SIZE_PX, CANVAS_SIZE_PX, fill=COLOR_RED_ALLIANCE, outline="")
        # Blue side marker (left border)
        self.canvas.create_rectangle(0, 0, 8, CANVAS_SIZE_PX, fill=COLOR_BLUE_ALLIANCE, outline="")

        # 5. Field Perimeter Wall
        self.canvas.create_rectangle(2, 2, CANVAS_SIZE_PX - 2, CANVAS_SIZE_PX - 2, outline=COLOR_FIELD_WALL, width=4)

        # 6. Coordinate Inch Labels on Canvas Corners
        self.canvas.create_text(18, CANVAS_SIZE_PX - 12, text="(0, 0)", fill="#64748b", font=("Consolas", 9))
        self.canvas.create_text(CANVAS_SIZE_PX - 28, CANVAS_SIZE_PX - 12, text="(144, 0)", fill="#64748b", font=("Consolas", 9))
        self.canvas.create_text(22, 14, text="(0, 144)", fill="#64748b", font=("Consolas", 9))
        self.canvas.create_text(CANVAS_SIZE_PX - 32, 14, text="(144, 144)", fill="#64748b", font=("Consolas", 9))

        # 7. Draw Trajectory Path Segments
        total_length_in = 0.0
        if len(self.waypoints) >= 2:
            for i in range(1, len(self.waypoints)):
                p0 = self.waypoints[i - 1]
                p1 = self.waypoints[i]

                if p1.motion_type == "bdrive":
                    # Sample Bezier curve
                    pts = sample_cubic_bezier((p0.x, p0.y), p0.heading, (p1.x, p1.y), p1.heading, reverse=p1.reverse)
                    # Accumulate distance
                    for k in range(1, len(pts)):
                        total_length_in += math.hypot(pts[k][0] - pts[k-1][0], pts[k][1] - pts[k-1][1])
                    # Draw path polyline
                    px_pts = []
                    for pt in pts:
                        cx, cy = field_to_canvas(pt[0], pt[1])
                        px_pts.extend([cx, cy])
                    if len(px_pts) >= 4:
                        # Glow shadow
                        self.canvas.create_line(*px_pts, fill=COLOR_PATH_SHADOW, width=6, capstyle="round")
                        # Center path line
                        self.canvas.create_line(*px_pts, fill=COLOR_PATH_LINE, width=3, capstyle="round")
                        # Midpoint direction arrow
                        mid_idx = len(pts) // 2
                        mcx, mcy = field_to_canvas(pts[mid_idx][0], pts[mid_idx][1])
                        self._draw_direction_arrow(mcx, mcy, pts[mid_idx][2])
                elif p1.motion_type == "drive":
                    cx0, cy0 = field_to_canvas(p0.x, p0.y)
                    cx1, cy1 = field_to_canvas(p1.x, p1.y)
                    total_length_in += math.hypot(p1.x - p0.x, p1.y - p0.y)
                    self.canvas.create_line(cx0, cy0, cx1, cy1, fill=COLOR_PATH_SHADOW, width=5, dash=(8, 4))
                    self.canvas.create_line(cx0, cy0, cx1, cy1, fill="#38bdf8", width=2, dash=(8, 4))
                    # Mid arrow
                    mcx, mcy = (cx0 + cx1) / 2.0, (cy0 + cy1) / 2.0
                    bearing = math.degrees(math.atan2(p1.x - p0.x, p1.y - p0.y))
                    self._draw_direction_arrow(mcx, mcy, bearing)
                else:
                    # In place turn
                    cx, cy = field_to_canvas(p1.x, p1.y)
                    self.canvas.create_oval(cx - 16, cy - 16, cx + 16, cy + 16, outline="#fbbf24", width=2, dash=(3, 3))

        # 8. Draw Waypoint Nodes & Robot Outlines
        for idx, wp in enumerate(self.waypoints):
            cx, cy = field_to_canvas(wp.x, wp.y)
            is_sel = (idx == self.selected_idx)

            # Draw Robot Chassis Outline at this waypoint
            self._draw_robot_chassis(cx, cy, wp.heading, is_selected=is_sel)

            # Node Color
            if idx == 0:
                node_color = COLOR_WAYPOINT_START
            elif idx == len(self.waypoints) - 1:
                node_color = COLOR_WAYPOINT_END
            else:
                node_color = COLOR_WAYPOINT_MID

            r = 12 if not is_sel else 14
            # Selection ring
            if is_sel:
                self.canvas.create_oval(cx - r - 4, cy - r - 4, cx + r + 4, cy + r + 4, outline=COLOR_WAYPOINT_SEL, width=3)

            # Waypoint Node Circle
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=node_color, outline="#ffffff", width=2)
            # Label
            self.canvas.create_text(cx, cy, text=str(idx + 1), fill="#ffffff", font=("Helvetica", 10, "bold"))

            # Heading Stalk & Knob
            h_len = 32
            rad = math.radians(wp.heading)
            hx = cx + h_len * math.sin(rad)
            hy = cy - h_len * math.cos(rad)
            self.canvas.create_line(cx, cy, hx, hy, fill=COLOR_HEADING_LINE, width=3)
            # Heading knob
            kr = 6 if not is_sel else 7
            self.canvas.create_oval(hx - kr, hy - kr, hx + kr, hy + kr, fill=COLOR_HEADING_LINE, outline="#ffffff", width=1.5)

        # Update stats
        est_time_s = (total_length_in / 24.0) + (len(self.waypoints) * 0.4)
        self.stats_lbl.configure(text=f"Length: {total_length_in:.1f} in | Time: ~{est_time_s:.1f} s | {len(self.waypoints)} Pts")

    def _draw_direction_arrow(self, cx: float, cy: float, heading_deg: float):
        """Draws small arrow head on canvas indicating path motion heading."""
        rad = math.radians(heading_deg)
        length = 10
        # Arrow tip
        ax = cx + length * math.sin(rad)
        ay = cy - length * math.cos(rad)
        # Left barb
        lx = cx - length * math.sin(rad) + (length / 2) * math.cos(rad)
        ly = cy + length * math.cos(rad) + (length / 2) * math.sin(rad)
        # Right barb
        rx = cx - length * math.sin(rad) - (length / 2) * math.cos(rad)
        ry = cy + length * math.cos(rad) - (length / 2) * math.sin(rad)
        self.canvas.create_polygon(ax, ay, lx, ly, rx, ry, fill="#38bdf8", outline="")

    def _draw_robot_chassis(self, cx: float, cy: float, heading_deg: float, is_selected: bool = False):
        """Draws scaled rectangular robot boundary box with front chevron."""
        w_px = (ROBOT_WIDTH_IN / 2.0) * SCALE
        l_px = (ROBOT_LENGTH_IN / 2.0) * SCALE
        rad = math.radians(heading_deg)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)

        corners = [
            (-w_px, -l_px),
            (w_px, -l_px),
            (w_px, l_px),
            (-w_px, l_px)
        ]
        poly = []
        for x, y in corners:
            # Heading 0 is +Y (up on field, negative cy)
            rx = cx + (x * cos_a + y * sin_a)
            ry = cy + (-x * sin_a + y * cos_a)
            poly.extend([rx, ry])

        box_color = "#334155" if not is_selected else "#475569"
        outline_color = "#64748b" if not is_selected else "#f59e0b"
        self.canvas.create_polygon(*poly, fill=box_color, outline=outline_color, width=1.5, stipple="gray25" if not is_selected else "")

        # Front direction chevron
        front_x = cx + l_px * math.sin(rad)
        front_y = cy - l_px * math.cos(rad)
        self.canvas.create_line(cx, cy, front_x, front_y, fill=COLOR_ROBOT_FRONT, width=2)

    # -------------------------------------------------------------------------
    # Mouse & Canvas Interactions
    # -------------------------------------------------------------------------
    def _on_canvas_press(self, event):
        cx, cy = event.x, event.y

        # Check if clicking a heading knob
        for idx, wp in enumerate(self.waypoints):
            wcx, wcy = field_to_canvas(wp.x, wp.y)
            rad = math.radians(wp.heading)
            hx = wcx + 32 * math.sin(rad)
            hy = wcy - 32 * math.cos(rad)
            if math.hypot(cx - hx, cy - hy) <= 12:
                self.selected_idx = idx
                self.drag_mode = "heading"
                self._draw_field()
                self._update_inspector()
                return

        # Check if clicking a waypoint node
        for idx, wp in enumerate(self.waypoints):
            wcx, wcy = field_to_canvas(wp.x, wp.y)
            if math.hypot(cx - wcx, cy - wcy) <= 18:
                self.selected_idx = idx
                self.drag_mode = "point"
                self._draw_field()
                self._update_inspector()
                return

        # Otherwise: clicked on empty space -> add new waypoint!
        fx, fy = canvas_to_field(cx, cy)
        # Inherit heading towards new point
        if self.waypoints:
            last = self.waypoints[-1]
            dx = fx - last.x
            dy = fy - last.y
            new_heading = math.degrees(math.atan2(dx, dy)) % 360.0 if math.hypot(dx, dy) > 1.0 else last.heading
        else:
            new_heading = 0.0

        new_wp = Waypoint(x=round(fx, 1), y=round(fy, 1), heading=round(new_heading, 1), motion_type="bdrive", speed=100.0)
        self.waypoints.append(new_wp)
        self.selected_idx = len(self.waypoints) - 1
        self.drag_mode = "point"

        self._draw_field()
        self._update_inspector()
        self._update_code_preview()

    def _on_canvas_drag(self, event):
        if self.selected_idx is None or self.drag_mode is None:
            return

        wp = self.waypoints[self.selected_idx]
        if self.drag_mode == "point":
            fx, fy = canvas_to_field(event.x, event.y)
            wp.x = round(fx, 1)
            wp.y = round(fy, 1)
        elif self.drag_mode == "heading":
            wcx, wcy = field_to_canvas(wp.x, wp.y)
            dx = event.x - wcx
            dy = wcy - event.y # Invert y since screen Y is down
            ang = math.degrees(math.atan2(dx, dy)) % 360.0
            wp.heading = round(ang, 1)

        self._draw_field()
        self._update_inspector()
        self._update_code_preview()

    def _on_canvas_release(self, event):
        self.drag_mode = None

    def _on_canvas_hover(self, event):
        fx, fy = canvas_to_field(event.x, event.y)
        tile_c = int(fx // TILE_SIZE_IN) + 1
        tile_r = int(fy // TILE_SIZE_IN) + 1
        self.coord_bar.configure(text=f"Cursor: X: {fx:.1f}\" | Y: {fy:.1f}\" | Tile: ({tile_c}, {tile_r})")

    def _on_canvas_right_click(self, event):
        # Quick delete on right click
        cx, cy = event.x, event.y
        for idx, wp in enumerate(self.waypoints):
            wcx, wcy = field_to_canvas(wp.x, wp.y)
            if math.hypot(cx - wcx, cy - wcy) <= 16:
                self.selected_idx = idx
                self._delete_selected()
                return

    # -------------------------------------------------------------------------
    # Inspector Sync
    # -------------------------------------------------------------------------
    def _update_inspector(self):
        if self.selected_idx is None or self.selected_idx >= len(self.waypoints):
            self.wp_label.configure(text="Waypoint: None Selected")
            self.entry_x.delete(0, "end")
            self.entry_y.delete(0, "end")
            self.entry_h.delete(0, "end")
            self.entry_wait.delete(0, "end")
            return

        wp = self.waypoints[self.selected_idx]
        total = len(self.waypoints)
        role = " (START)" if self.selected_idx == 0 else (" (END)" if self.selected_idx == total - 1 else "")
        self.wp_label.configure(text=f"Waypoint #{self.selected_idx + 1} of {total}{role}")

        # Update entries without triggering callback
        for entry, val in [(self.entry_x, f"{wp.x:.1f}"), (self.entry_y, f"{wp.y:.1f}"), (self.entry_h, f"{wp.heading:.1f}")]:
            entry.delete(0, "end")
            entry.insert(0, val)

        self.motion_seg.set(wp.motion_type)
        if wp.reverse:
            self.reverse_chk.select()
        else:
            self.reverse_chk.deselect()

        self.speed_slider.set(wp.speed)
        self.speed_lbl.configure(text=f"{int(wp.speed)}%")

        self.action_opt.set(wp.action)
        self.entry_wait.delete(0, "end")
        self.entry_wait.insert(0, str(wp.wait_ms))

    def _on_coord_edited(self):
        if self.selected_idx is None:
            return
        wp = self.waypoints[self.selected_idx]
        try:
            wp.x = max(0.0, min(FIELD_SIZE_IN, float(self.entry_x.get())))
            wp.y = max(0.0, min(FIELD_SIZE_IN, float(self.entry_y.get())))
            wp.heading = normalize_angle(float(self.entry_h.get()))
            wp.wait_ms = max(0, int(self.entry_wait.get()))
        except ValueError:
            pass
        self._draw_field()
        self._update_inspector()
        self._update_code_preview()

    def _set_heading_quick(self, angle: float):
        if self.selected_idx is None:
            return
        self.waypoints[self.selected_idx].heading = float(angle)
        self._draw_field()
        self._update_inspector()
        self._update_code_preview()

    def _on_motion_type_changed(self, val: str):
        if self.selected_idx is None:
            return
        self.waypoints[self.selected_idx].motion_type = val
        self._draw_field()
        self._update_code_preview()

    def _on_reverse_toggled(self):
        if self.selected_idx is None:
            return
        self.waypoints[self.selected_idx].reverse = bool(self.reverse_chk.get())
        self._draw_field()
        self._update_code_preview()

    def _on_speed_changed(self, val: float):
        if self.selected_idx is None:
            return
        spd = round(val)
        self.waypoints[self.selected_idx].speed = spd
        self.speed_lbl.configure(text=f"{spd}%")
        self._update_code_preview()

    def _on_action_changed(self, val: str):
        if self.selected_idx is None:
            return
        self.waypoints[self.selected_idx].action = val
        self._update_code_preview()

    def _delete_selected(self):
        if self.selected_idx is None or not self.waypoints:
            return
        del self.waypoints[self.selected_idx]
        if self.waypoints:
            self.selected_idx = min(self.selected_idx, len(self.waypoints) - 1)
        else:
            self.selected_idx = None
        self._draw_field()
        self._update_inspector()
        self._update_code_preview()

    def _clear_path(self):
        if messagebox.askyesno("Clear Path", "Reset path to default starting position?"):
            self.waypoints = [Waypoint(x=72.0, y=18.0, heading=0.0)]
            self.selected_idx = 0
            self._draw_field()
            self._update_inspector()
            self._update_code_preview()

    def _mirror_path(self):
        """Mirrors the entire path across X=72 (Left/Right field flip)."""
        if not self.waypoints:
            return
        for wp in self.waypoints:
            wp.x = round(FIELD_SIZE_IN - wp.x, 1)
            wp.heading = round((360.0 - wp.heading) % 360.0, 1)

        # Toggle auton dropdown between RS and LS
        current = self.auton_dropdown.get()
        if current == "RS":
            self.auton_dropdown.set("LS")
        elif current == "LS":
            self.auton_dropdown.set("RS")

        self._draw_field()
        self._update_inspector()
        self._update_code_preview()

    # -------------------------------------------------------------------------
    # Auton Selection & Code Export
    # -------------------------------------------------------------------------
    def _on_auton_select(self, val: str):
        self._update_code_preview()

    def _update_code_preview(self):
        routine = self.auton_dropdown.get()
        code = generate_cpp_code(self.waypoints, routine)
        self.code_text.delete("0.0", "end")
        self.code_text.insert("0.0", code)

    def _copy_code(self):
        code = self.code_text.get("0.0", "end")
        self.clipboard_clear()
        self.clipboard_append(code)
        messagebox.showinfo("Copied", "Autonomous C++ code copied to clipboard!")

    def _write_to_codebase(self):
        routine = self.auton_dropdown.get()
        code = generate_cpp_code(self.waypoints, routine)

        success, msg = write_code_to_autons_cpp(self.project_root, routine, code)
        if success:
            # Flash button green
            orig_text = self.write_btn.cget("text")
            self.write_btn.configure(text="✔ WRITTEN TO SRC/AUTONS.CPP!", fg_color="#059669")
            self.after(2000, lambda: self.write_btn.configure(text=orig_text, fg_color="#10b981"))
            messagebox.showinfo("Success", f"{msg}\n\nYou can now compile or run your autonomous routine!")
        else:
            messagebox.showerror("Write Error", msg)

    # -------------------------------------------------------------------------
    # Path File Save / Load
    # -------------------------------------------------------------------------
    def _save_path_dialog(self):
        default_dir = os.path.join(self.project_root, "paths")
        os.makedirs(default_dir, exist_ok=True)
        routine = self.auton_dropdown.get()
        file_path = filedialog.asksaveasfilename(
            initialdir=default_dir,
            initialfile=f"{routine}_path.json",
            defaultextension=".json",
            filetypes=[("JSON Path Files", "*.json")]
        )
        if file_path:
            data = {
                "name": f"{routine} Path",
                "routine": routine,
                "waypoints": [w.to_dict() for w in self.waypoints]
            }
            with open(file_path, "w") as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Saved", f"Path saved to {os.path.basename(file_path)}")

    def _load_path_dialog(self):
        default_dir = os.path.join(self.project_root, "paths")
        file_path = filedialog.askopenfilename(
            initialdir=default_dir,
            filetypes=[("JSON Path Files", "*.json")]
        )
        if file_path:
            try:
                with open(file_path, "r") as f:
                    data = json.load(f)
                    self.waypoints = [Waypoint.from_dict(w) for w in data["waypoints"]]
                    if "routine" in data:
                        self.auton_dropdown.set(data["routine"])
                    self.selected_idx = 0 if self.waypoints else None
                    self._draw_field()
                    self._update_inspector()
                    self._update_code_preview()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load path: {str(e)}")

    # -------------------------------------------------------------------------
    # Simulation Player
    # -------------------------------------------------------------------------
    def _toggle_simulation(self):
        if self.sim_active:
            self.sim_active = False
            self.play_btn.configure(text="▶ Play Sim", fg_color="#0284c7")
            self._draw_field()
        else:
            if len(self.waypoints) < 2:
                messagebox.showwarning("Simulation", "Please add at least 2 waypoints to simulate.")
                return
            self.sim_active = True
            self.play_btn.configure(text="⏹ Stop", fg_color="#ef4444")
            self._run_simulation()

    def _run_simulation(self):
        # Generate full trajectory points
        full_pts = []
        for i in range(1, len(self.waypoints)):
            p0 = self.waypoints[i - 1]
            p1 = self.waypoints[i]
            if p1.motion_type == "bdrive":
                pts = sample_cubic_bezier((p0.x, p0.y), p0.heading, (p1.x, p1.y), p1.heading, reverse=p1.reverse, n_samples=30)
                full_pts.extend(pts[:-1])
            else:
                pts = [(p0.x, p0.y, p0.heading), (p1.x, p1.y, p1.heading)]
                full_pts.extend(pts[:-1])
        full_pts.append((self.waypoints[-1].x, self.waypoints[-1].y, self.waypoints[-1].heading))

        speed_mult = float(self.sim_speed_opt.get().replace("x", ""))
        delay_ms = int(max(15, 40 / speed_mult))

        def step(idx):
            if not self.sim_active:
                return
            if idx >= len(full_pts):
                self.sim_active = False
                self.play_btn.configure(text="▶ Play Sim", fg_color="#0284c7")
                self._draw_field()
                return

            self._draw_field()
            # Draw moving simulated robot
            pt = full_pts[idx]
            cx, cy = field_to_canvas(pt[0], pt[1])
            self._draw_robot_chassis(cx, cy, pt[2], is_selected=True)
            # Glowing ghost marker
            self.canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#38bdf8", outline="#ffffff", width=2)
            self.after(delay_ms, lambda: step(idx + 1))

        step(0)


# =============================================================================
# ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    app = PathStudioApp()
    app.mainloop()
