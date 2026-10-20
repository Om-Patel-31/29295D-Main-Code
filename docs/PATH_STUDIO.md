# 29295D Path Studio — User Guide

**29295D Path Studio** is a standalone visual path planner and C++ code generator inspired by `path.jerryio.com`. It allows you to visually design autonomous paths on a top-down VEX field and write them directly into your robot's code without having to manually write C++ commands.

---

## 🚀 Quick Start

### How to Launch
Double-click `run_path_planner.bat` or run:
```bash
python path_planner.py
```

---

## 🎨 Interactive Controls & Features

### 1. Drawing & Editing Paths
- **Add Waypoint:** Click anywhere on the 144" x 144" VEX field to create a new waypoint.
- **Move Waypoint:** Click and drag any waypoint node to adjust its position.
- **Rotate Heading:** Click and drag the yellow heading handle extending from each waypoint to dynamically rotate the robot's heading.
- **Select Waypoint:** Click on any waypoint to inspect and edit its coordinates, heading, motion type, and actions in the sidebar.
- **Delete Waypoint:** Right-click on a waypoint or click the **Delete** button in the sidebar.

### 2. Motion Types
In the sidebar **Motion** selector, you can choose:
- **`bdrive` (Cubic Bezier Drive):** Generates smooth, continuous-curvature trajectories between waypoints using your robot's LADRC path tracker.
- **`drive` (Straight Line Drive):** Drives straight from point A to point B, turning if needed to align with the heading.
- **`turn` (Turn in Place):** Rotates the robot in place to the target heading.

### 3. Waypoint Actions & Modifiers
- **Speed Slider (10% - 100%):** Sets the maximum velocity limit for that motion segment.
- **Reverse Checkbox:** Makes the robot traverse the segment in reverse (with the back of the robot leading).
- **Actions:**
  - `Clamp Claw`: Automatically adds claw clamp commands.
  - `Open Claw`: Automatically adds claw release commands.
  - `Lift Up` / `Lift Down`: Commands the cascade lift.
  - `sdrive`: Actively brakes and stops the drivetrain.
  - `Wait`: Adds a custom delay pause (in milliseconds).

### 4. 1-Click Left/Right Mirroring (`⇄ Mirror`)
Clicking **`⇄ Mirror (X-Flip)`** instantly mirrors your entire path across the center of the field ($X = 72"$) and inverts headings. This allows you to design your Right Side (`RS`) auton once and convert it to Left Side (`LS`) with a single click!

### 5. Trajectory Simulation (`▶ Play Sim`)
Click **`▶ Play Sim`** to see an animated robot follow your path in real time, with playback speeds from `0.5x` to `4x`.

---

## ⚡ Exporting to C++ Autonomous Code

1. In the **Target Auton** dropdown, select the routine you want to update:
   - `RS` (Right Side Match Auton)
   - `LS` (Left Side Match Auton)
   - `Skills` (Programming Skills)
2. Review the live generated C++ code in the preview box.
3. Click the **`⚡ WRITE TO AUTONS.CPP`** button.
4. Path Studio will automatically update the selected function inside [`src/autons.cpp`](file:///c:/Users/om31d/Documents/vex-vscode-projects/29295D-Main-Code/src/autons.cpp) with full inline comments and create a backup at `src/autons.cpp.bak`.

---

## 💾 Saving & Loading Paths
- Click **`💾 Save JSON`** to save your path to `paths/` for later editing.
- Click **`📂 Load JSON`** to load any previously saved path or default preset.

