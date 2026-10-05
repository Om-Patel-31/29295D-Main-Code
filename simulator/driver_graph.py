import sys
import os
import re
import math
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QFileDialog, QMessageBox, 
                             QLineEdit, QFrame, QSlider, QDoubleSpinBox)
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QBrush, QFont
from PyQt6.QtCore import Qt, QRect, QPoint

# --- Constants ---
CANVAS_SIZE = 500
ACCENT_COLOR = QColor(0, 255, 255)
BG_COLOR = QColor(25, 25, 35)
PANEL_COLOR = QColor(45, 45, 60)
TEXT_COLOR = QColor(230, 230, 230)

class CurveCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(CANVAS_SIZE, CANVAS_SIZE)
        self.deadzone = 0.1
        self.exponent = 2.0
        
    def set_params(self, dz, exp):
        self.deadzone = dz
        self.exponent = exp
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Background
        painter.fillRect(self.rect(), QBrush(BG_COLOR))
        
        # Draw Axes
        painter.setPen(QPen(QColor(80, 80, 100), 2))
        painter.drawLine(0, CANVAS_SIZE//2, CANVAS_SIZE, CANVAS_SIZE//2) # X axis
        painter.drawLine(CANVAS_SIZE//2, 0, CANVAS_SIZE//2, CANVAS_SIZE) # Y axis
        
        # Labels
        painter.setPen(QPen(TEXT_COLOR))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(CANVAS_SIZE-40, CANVAS_SIZE//2 - 15, "Input (100)")
        painter.drawText(CANVAS_SIZE//2 + 10, 20, "Output (100)")
        painter.drawText(10, CANVAS_SIZE//2 + 10, "Input (-100)")
        painter.drawText(CANVAS_SIZE//2 - 90, CANVAS_SIZE-20, "Output (-100)")

        # Deadzone visualization
        dz_pixels = self.deadzone * (CANVAS_SIZE // 2)
        painter.setBrush(QBrush(QColor(255, 0, 0, 30)))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRect(int(CANVAS_SIZE//2 - dz_pixels), 0, int(2*dz_pixels), CANVAS_SIZE)

        # Plot the curve
        painter.setPen(QPen(ACCENT_COLOR, 4))
        
        path = []
        for i in range(0, CANVAS_SIZE + 1, 2):
            nx = (i - CANVAS_SIZE//2) / (CANVAS_SIZE // 2)
            if abs(nx) < self.deadzone:
                ny = 0
            else:
                sign = 1 if nx > 0 else -1
                scaled_nx = (abs(nx) - self.deadzone) / (1.0 - self.deadzone)
                ny = sign * (scaled_nx ** self.exponent)
            
            px = i
            py = int(CANVAS_SIZE//2 - (ny * (CANVAS_SIZE // 2)))
            path.append(QPoint(px, py))
        
        for i in range(len(path)-1):
            painter.drawLine(path[i], path[i+1])

class DriverGraphApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VEX Driver Curve Tuner Pro")
        self.resize(850, 650)
        self.setStyleSheet(f"QMainWindow {{ background-color: {BG_COLOR.name()}; color: {TEXT_COLOR.name()}; }}")
        
        main_widget = QWidget()
        main_widget.setStyleSheet(f"background-color: {BG_COLOR.name()};")
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(40)

        # --- Left Side: Graph ---
        self.curve_canvas = CurveCanvas()
        layout.addWidget(self.curve_canvas)

        # --- Right Side: Controls ---
        controls_panel = QFrame()
        controls_panel.setFixedWidth(350)
        controls_panel.setStyleSheet(f"background-color: {PANEL_COLOR.name()}; border-radius: 15px; padding: 20px;")
        controls_layout = QVBoxLayout(controls_panel)
        controls_layout.setSpacing(15)

        # Header
        header = QLabel("Curve Configuration")
        header.setStyleSheet("font-size: 20px; font-weight: bold; color: white; margin-bottom: 10px;")
        controls_layout.addWidget(header)

        # Deadzone Section
        dz_frame = QFrame()
        dz_frame.setStyleSheet("background: transparent;")
        dz_layout = QVBoxLayout(dz_frame)
        
        dz_label = QLabel("Deadzone")
        dz_label.setStyleSheet("font-weight: bold; color: #AAA;")
        dz_layout.addWidget(dz_label)
        
        dz_row = QHBoxLayout()
        self.slider_dz = QSlider(Qt.Orientation.Horizontal)
        self.slider_dz.setRange(0, 50)
        self.slider_dz.setValue(10)
        self.slider_dz.valueChanged.connect(self.on_slider_dz_changed)
        
        self.spin_dz = QDoubleSpinBox()
        self.spin_dz.setRange(0.0, 0.5)
        self.spin_dz.setSingleStep(0.01)
        self.spin_dz.setValue(0.1)
        self.spin_dz.valueChanged.connect(self.on_spin_dz_changed)
        
        dz_row.addWidget(self.slider_dz)
        dz_row.addWidget(self.spin_dz)
        dz_layout.addWidget(QWidget()) # Spacer
        dz_layout.addLayout(dz_row)
        
        dz_desc = QLabel("Input range around 0 that results in 0 output. Prevents 'stick drift'.")
        dz_desc.setStyleSheet("font-size: 11px; color: #888;")
        dz_layout.addWidget(dz_desc)
        controls_layout.addWidget(dz_frame)

        # Exponent Section
        exp_frame = QFrame()
        exp_frame.setStyleSheet("background: transparent;")
        exp_layout = QVBoxLayout(exp_frame)
        
        exp_label = QLabel("Power Exponent")
        exp_label.setStyleSheet("font-weight: bold; color: #AAA;")
        exp_layout.addWidget(exp_label)
        
        exp_row = QHBoxLayout()
        self.slider_exp = QSlider(Qt.Orientation.Horizontal)
        self.slider_exp.setRange(10, 50)
        self.slider_exp.setValue(20)
        self.slider_exp.valueChanged.connect(self.on_slider_exp_changed)
        
        self.spin_exp = QDoubleSpinBox()
        self.spin_exp.setRange(1.0, 5.0)
        self.spin_exp.setSingleStep(0.1)
        self.spin_exp.setValue(2.0)
        self.spin_exp.valueChanged.connect(self.on_spin_exp_changed)
        
        exp_row.addWidget(self.slider_exp)
        exp_row.addWidget(self.spin_exp)
        exp_layout.addLayout(exp_row)
        
        exp_desc = QLabel("Determines the 'curve'. 1.0 is linear. Higher values make low input more precise.")
        exp_desc.setStyleSheet("font-size: 11px; color: #888;")
        exp_layout.addWidget(exp_desc)
        controls_layout.addWidget(exp_frame)

        controls_layout.addStretch()
        
        self.btn_save = QPushButton("💾 Export to tuning.cpp")
        self.btn_save.clicked.connect(self.save_curve_to_cpp)
        self.btn_save.setStyleSheet(f"""
            QPushButton {{
                background-color: {ACCENT_COLOR.name()}; 
                color: {BG_COLOR.name()}; 
                font-weight: bold; 
                font-size: 14px;
                height: 50px; 
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: #00cccc;
            }}
        """)
        controls_layout.addWidget(self.btn_save)
        
        layout.addWidget(controls_panel)
        self.update_curve()

    def on_slider_dz_changed(self):
        val = self.slider_dz.value() / 100.0
        self.spin_dz.setValue(val)
        self.update_curve()

    def on_spin_dz_changed(self, val):
        self.slider_dz.setValue(int(val * 100))
        self.update_curve()

    def on_slider_exp_changed(self):
        val = self.slider_exp.value() / 10.0
        self.spin_exp.setValue(val)
        self.update_curve()

    def on_spin_exp_changed(self, val):
        self.slider_exp.setValue(int(val * 10))
        self.update_curve()

    def update_curve(self):
        dz = self.spin_dz.value()
        exp = self.spin_exp.value()
        self.curve_canvas.set_params(dz, exp)

    def save_curve_to_cpp(self):
        dz = self.spin_dz.value() * 100.0
        exp = self.spin_exp.value()
        
        code = f'''
// Auto-generated by VEX Driver Graph Tuner Pro
double get_curved_input(double input) {{
    double deadzone = {dz:.3f};
    double exponent = {exp:.3f};
    
    if (std::abs(input) < deadzone) return 0.0;
    
    double sign = (input > 0) ? 1.0 : -1.0;
    double scaled = (std::abs(input) - deadzone) / (100.0 - deadzone);
    return sign * (std::pow(scaled, exponent) * 100.0);
}}
'''
        try:
            file_path = os.path.join(os.getcwd(), "src", "tuning.cpp")
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'w') as f:
                f.write(code)
            QMessageBox.information(self, "Success", f"Curve settings saved to {file_path}!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {str(e)}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DriverGraphApp()
    window.show()
    sys.exit(app.exec())
