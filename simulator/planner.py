import sys
import os
import re
import math
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QComboBox, QLabel, QFileDialog, QMessageBox, 
                             QTabWidget, QListWidget, QLineEdit, QFormLayout, QFrame, QSlider, QDoubleSpinBox, QCheckBox)
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QBrush, QFont
from PyQt6.QtCore import Qt, QRect, QPoint, pyqtSignal

# --- Constants ---
DEFAULT_GRID_SIZE = 800
DEFAULT_PPI = 10.0

class PathSegment:
    def __init__(self, seg_type="Straight", points=None, params=None):
        self.type = seg_type
        self.points = points or []
        self.params = params or {}

    def __str__(self):
        return f"{self.type} ({len(self.points)} pts)"

class FieldCanvas(QWidget):
    point_added = pyqtSignal(QPoint)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.image = QPixmap()
        self.waypoints = []
        self.pixel_to_inch = DEFAULT_PPI
        self.grid_size = DEFAULT_GRID_SIZE
        self.setFixedSize(self.grid_size, self.grid_size)

    def load_image(self, path):
        loaded_image = QPixmap(path)
        if not loaded_image.isNull():
            self.image = loaded_image.scaled(
                self.grid_size, self.grid_size, 
                Qt.AspectRatioMode.IgnoreAspectRatio, 
                Qt.TransformationMode.SmoothTransformation
            )
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            p = QPoint(int(event.position().x()), int(event.position().y()))
            self.waypoints.append(p)
            self.point_added.emit(p)
            self.update()
        elif event.button() == Qt.MouseButton.RightButton:
            self.waypoints = []
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        if not self.image.isNull():
            painter.drawPixmap(0, 0, self.image)
        if not self.waypoints:
            return
        painter.setPen(QPen(QColor(0, 255, 0), 3))
        for i in range(len(self.waypoints)):
            p = self.waypoints[i]
            painter.drawEllipse(p.x()-4, p.y()-4, 8, 8)
            if i > 0:
                prev = self.waypoints[i-1]
                painter.drawLine(prev, p)

    def get_coordinates(self):
        cx, cy = self.grid_size / 2, self.grid_size / 2
        coords = []
        for p in self.waypoints:
            x = (p.x() - cx) / self.pixel_to_inch
            y = (cy - p.y()) / self.pixel_to_inch
            coords.append((x, y))
        return coords

class AccelGraphCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(600, 300)
        self.data_history = [] # List of (time, value)
        self.max_history = 100
        self.y_min = -50.0
        self.y_max = 50.0

    def add_sample(self, value):
        self.data_history.append(value)
        if len(self.data_history) > self.max_history:
            self.data_history.pop(0)
        self.update()

    def set_range(self, ymin, ymax):
        self.y_min = ymin
        self.y_max = ymax
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QBrush(QColor(30, 30, 30)))
        
        # Draw Axes
        painter.setPen(QPen(QColor(100, 100, 100), 1))
        painter.drawLine(0, self.height()//2, self.width(), self.height()//2)
        painter.drawLine(self.width()//2, 0, self.width()//2, self.height())
        
        if len(self.data_history) < 2:
            return

        painter.setPen(QPen(QColor(0, 255, 255), 2))
        dx = self.width() / (self.max_history - 1)
        
        for i in range(len(self.data_history)-1):
            v1 = self.data_history[i]
            v2 = self.data_history[i+1]
            
            # Map value to pixels
            # y = height/2 - (value - min) / (max - min) * height
            y1 = self.height()//2 - ((v1 - self.y_min) / (self.y_max - self.y_min)) * self.height()
            y2 = self.height()//2 - ((v2 - self.y_min) / (self.y_max - self.y_min)) * self.height()
            
            painter.drawLine(int(i * dx), int(y1), int((i+1) * dx), int(y2))

class PathPlanner(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VEX Pro Path Planner & Accel Tuner")
        self.resize(1300, 900)
        
        self.segments = [] 
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # --- Left Panel: Toolset ---
        left_panel = QFrame()
        left_panel.setFixedWidth(320)
        left_panel.setFrameShape(QFrame.Shape.StyledPanel)
        left_layout = QVBoxLayout(left_panel)

        self.btn_load = QPushButton("📁 Load Field Image")
        self.btn_load.clicked.connect(self.load_field)
        left_layout.addWidget(self.btn_load)

        left_layout.addWidget(QLabel("\n<b style='font-size:12px;'>Segment Type:</b>"))
        self.combo_type = QComboBox()
        self.combo_type.addItems(["Straight", "Bezier", "Turn"])
        self.combo_type.currentTextChanged.connect(self.on_type_changed)
        left_layout.addWidget(self.combo_type)

        self.param_container = QWidget()
        self.param_layout = QFormLayout(self.param_container)
        left_layout.addWidget(self.param_container)

        left_layout.addWidget(QLabel("\n<b style='font-size:12px;'>Scaling:</b>"))
        self.edit_ppi = QLineEdit(str(DEFAULT_PPI))
        self.edit_ppi.textChanged.connect(self.update_ppi)
        left_layout.addWidget(QLabel("Pixels per Inch:"))
        left_layout.addWidget(self.edit_ppi)

        left_layout.addWidget(QLabel("\n<b style='font-size:12px;'>Path Library:</b>"))
        self.path_list = QListWidget()
        left_layout.addWidget(self.path_list)

        self.btn_add_seg = QPushButton("➕ Add Segment to Path")
        self.btn_add_seg.clicked.connect(self.add_current_segment)
        left_layout.addWidget(self.btn_add_seg)

        self.btn_clear_all = QPushButton("🗑️ Clear All Segments")
        self.btn_clear_all.clicked.connect(self.clear_segments)
        left_layout.addWidget(self.btn_clear_all)

        left_layout.addStretch()

        export_frame = QFrame()
        export_frame.setFrameShape(QFrame.Shape.StyledPanel)
        export_layout = QVBoxLayout(export_frame)
        export_layout.addWidget(QLabel("<b style='font-size:12px;'>Export to Auton:</b>"))
        self.combo_auton = QComboBox()
        self.combo_auton.addItems(["RS", "LS", "Skills", "Test_auton"])
        export_layout.addWidget(self.combo_auton)

        self.btn_write = QPushButton("🚀 Write to autons.cpp")
        self.btn_write.clicked.connect(self.write_to_file)
        self.btn_write.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold; height: 40px;")
        export_layout.addWidget(self.btn_write)
        left_layout.addWidget(export_frame)

        main_layout.addWidget(left_panel)

        # --- Center Panel: Tabs ---
        self.tabs = QTabWidget()
        
        # Tab 1: Planner
        self.planner_tab = QWidget()
        planner_layout = QVBoxLayout(self.planner_tab)
        self.canvas = FieldCanvas()
        planner_layout.addWidget(self.canvas, alignment=Qt.AlignmentFlag.AlignCenter)
        self.tabs.addTab(self.planner_tab, "🗺️ Path Planner")

        # Tab 2: Accel Diagnostics (The new feature)
        self.accel_tab = QWidget()
        accel_layout = QHBoxLayout(self.accel_tab)
        
        # Accel Graph
        self.accel_graph = AccelGraphCanvas()
        accel_layout.addWidget(self.accel_graph)
        
        # Accel Controls
        accel_controls = QVBoxLayout()
        accel_controls.addWidget(QLabel("<b style='font-size:14px;'>Accel Settings</b>"))
        
        self.chk_show_accel = QCheckBox("Show Acceleration") # Note: QCheckBox needs import
        self.chk_show_accel.setChecked(True)
        accel_controls.addWidget(self.chk_show_accel)
        
        accel_controls.addWidget(QLabel("Y-Axis Max:"))
        self.spin_y_max = QDoubleSpinBox()
        self.spin_y_max.setRange(1.0, 500.0)
        self.spin_y_max.setValue(50.0)
        self.spin_y_max.valueChanged.connect(self.update_accel_range)
        accel_controls.addWidget(self.spin_y_max)
        
        accel_controls.addWidget(QLabel("Smoothing (EMA alpha):"))
        self.spin_smooth = QDoubleSpinBox()
        self.spin_smooth.setRange(0.01, 1.0)
        self.spin_smooth.setSingleStep(0.05)
        self.spin_smooth.setValue(0.2)
        accel_controls.addWidget(self.spin_smooth)
        
        accel_controls.addStretch()
        accel_layout.addLayout(accel_controls)
        
        self.tabs.addTab(self.accel_tab, "📈 Accel Diagnostics")
        
        main_layout.addWidget(self.tabs)

        self.on_type_changed(self.combo_type.currentText())

    def on_type_changed(self, seg_type):
        while self.param_layout.count():
            child = self.param_layout.takeAt(0)
            if child.widget():
                child.widget().setParent(None)
        if seg_type == "Straight":
            self.param_layout.addRow("Description:", QLineEdit("Forward drive"))
        elif seg_type == "Bezier":
            self.param_layout.addRow("Curvature:", QLineEdit("1.0"))
            self.param_layout.addRow("Tension:", QLineEdit("0.5"))
        elif seg_type == "Turn":
            self.param_layout.addRow("Angle (deg):", QLineEdit("90"))
            self.param_layout.addRow("Speed:", QLineEdit("50"))

    def load_field(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Field Image", "", "Images (*.png *.jpg *.jpeg)")
        if path:
            self.canvas.load_image(path)

    def update_ppi(self, text):
        try:
            self.canvas.pixel_to_inch = float(text)
            self.canvas.update()
        except ValueError:
            pass

    def update_accel_range(self):
        val = self.spin_y_max.value()
        self.accel_graph.set_range(-val, val)

    def add_current_segment(self):
        seg_type = self.combo_type.currentText()
        pts = self.canvas.get_coordinates()
        params = {}
        for i in range(self.param_layout.rowCount()):
            widget = self.param_layout.itemAt(i).widget()
            if isinstance(widget, QLineEdit):
                label_widget = self.param_layout.itemAt(i-1).widget() if i > 0 else None
                label = label_widget.text() if label_widget else f"p{i}"
                params[label] = widget.text()
        seg = PathSegment(seg_type, pts, params)
        self.segments.append(seg)
        self.path_list.addItem(str(seg))
        self.canvas.waypoints = []
        self.canvas.update()

    def clear_segments(self):
        self.segments = []
        self.path_list.clear()

    def write_to_file(self):
        auton_name = self.combo_auton.currentText()
        if not self.segments:
            QMessageBox.warning(self, "Error", "No segments added to path!")
            return
        code_lines = []
        for seg in self.segments:
            if seg.type == "Straight":
                for x, y in seg.points:
                    code_lines.append(f"    move_to({x:.2f}, {y:.2f});")
            elif seg.type == "Bezier":
                code_lines.append(f"    // Bezier Path segment")
                for x, y in seg.points:
                    code_lines.append(f"    move_to_bezier({x:.2f}, {y:.2f});")
            elif seg.type == "Turn":
                angle = seg.params.get("Angle (deg):", "90")
                code_lines.append(f"    turn_robot({angle});")
        generated_code = "\n".join(code_lines)
        file_path = os.path.join(os.getcwd(), "src", "autons.cpp")
        try:
            with open(file_path, 'r') as f:
                content = f.read()
            pattern = rf"(void\s+{auton_name}\s*\(\)\s*\{{)(.*?)(}})"
            replacement = rf"\1\n    // Path generated by Pro Planner\n{generated_code}\n\n\3"
            new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)
            with open(file_path, 'w') as f:
                f.write(new_content)
            QMessageBox.information(self, "Success", f"Path exported to {auton_name}!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to write: {str(e)}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PathPlanner()
    window.show()
    sys.exit(app.exec())
