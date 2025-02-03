import sys
import cv2
import numpy as np
import open3d as o3d
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, 
                           QWidget, QComboBox, QHBoxLayout, QLabel, QFileDialog)
from PyQt5.QtCore import QTimer, Qt, QPointF
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen, QPalette  # Added QPalette here
from PyQt5.QtWidgets import QOpenGLWidget  # Changed import
from OpenGL.GL import *
from OpenGL.GLU import *
from scipy.spatial.transform import Rotation
from PIL import Image
import math

class CameraPreviewWidget(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 240)
        self.setAlignment(Qt.AlignCenter)

    def update_frame(self, frame):
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_BGR888)
        scaled_pixmap = QPixmap.fromImage(qt_image).scaled(self.size(), Qt.KeepAspectRatio)
        self.setPixmap(scaled_pixmap)

class SphereVisWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.coverage_data = []
        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, Qt.white)  # Updated to use proper enum
        self.setPalette(palette)
        self.setAutoFillBackground(True)

    def update_coverage(self, coverage_points):
        self.coverage_data = coverage_points
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw sphere
        center = QPointF(self.width() / 2, self.height() / 2)
        radius = min(self.width(), self.height()) / 2.2

        # Draw basic sphere outline
        painter.setPen(QPen(Qt.black, 1))
        painter.drawEllipse(center, radius, radius)

        # Draw coverage points
        for point, covered in self.coverage_data:
            x = center.x() + radius * math.cos(point[0]) * math.cos(point[1])
            y = center.y() + radius * math.sin(point[1])
            
            color = QColor(0, 255, 0) if covered else QColor(255, 0, 0)
            painter.setPen(QPen(color, 3))
            painter.drawPoint(x, y)

class ScannerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.scanning = False
        self.available_cameras = self.get_available_cameras()
        self.camera_preview = None
        self.vis_widget = None
        self.cap = None
        self.frames = []
        self.frame_positions = []
        self.sphere_vertices = None
        self.coverage_map = None
        self.init_coverage_tracking()
        self.sphere_points = self.generate_sphere_points()
        self.setup_ui()  # Move after initializing all instance variables

    def get_available_cameras(self):
        """Detect available camera devices"""
        available_cameras = []
        for i in range(10):  # Check first 10 indexes
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    available_cameras.append(f"Camera {i}")
                cap.release()
        return available_cameras

    def setup_ui(self):
        self.setWindowTitle('360° Scanner')
        self.setGeometry(100, 100, 1200, 800)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)
        
        # Left panel for controls and camera preview
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        # Camera controls
        camera_layout = QHBoxLayout()
        camera_label = QLabel("Select Camera:")
        self.camera_combo = QComboBox()
        self.camera_combo.addItems(self.available_cameras)
        camera_layout.addWidget(camera_label)
        camera_layout.addWidget(self.camera_combo)
        left_layout.addLayout(camera_layout)
        
        # Initialize camera preview before adding to layout
        self.camera_preview = CameraPreviewWidget()
        if not self.camera_preview:
            raise RuntimeError("Failed to initialize camera preview widget")
        left_layout.addWidget(self.camera_preview)
        
        # Add scan button
        self.scan_button = QPushButton('Start Scanning')
        self.scan_button.clicked.connect(self.toggle_scanning)
        left_layout.addWidget(self.scan_button)
        
        # Right panel for visualization
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # Initialize visualization widget before adding to layout
        self.vis_widget = SphereVisWidget()
        if not self.vis_widget:
            raise RuntimeError("Failed to initialize visualization widget")
        right_layout.addWidget(self.vis_widget)
        
        # Add panels to main layout
        layout.addWidget(left_panel, 1)
        layout.addWidget(right_panel, 2)  # Give visualization more space
        
    def init_coverage_tracking(self):
        # Create higher resolution sphere for better coverage tracking
        sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=50)
        self.sphere_vertices = np.asarray(sphere_mesh.vertices)
        # Initialize coverage map (0 = not covered, 1 = covered)
        self.coverage_map = np.zeros(len(self.sphere_vertices))
        
    def generate_sphere_points(self, num_points=200):
        points = []
        phi = math.pi * (3 - math.sqrt(5))  # Golden angle
        
        for i in range(num_points):
            y = 1 - (i / float(num_points - 1)) * 2
            radius = math.sqrt(1 - y * y)
            theta = phi * i
            
            x = math.cos(theta) * radius
            z = math.sin(theta) * radius
            
            # Store as spherical coordinates (theta, phi)
            theta = math.atan2(z, x)
            phi = math.acos(y)
            points.append(((theta, phi), False))
            
        return points

    def toggle_scanning(self):
        self.scanning = not self.scanning
        if self.scanning:
            # Get selected camera index
            camera_idx = int(self.camera_combo.currentText().split()[-1])
            self.scan_button.setText('Stop Scanning')
            self.cap = cv2.VideoCapture(camera_idx)
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_scan)
            self.timer.start(30)  # 30ms refresh rate
            self.frames = []  # Clear previous frames
            self.frame_positions = []
        else:
            self.scan_button.setText('Start Scanning')
            if self.cap:
                self.cap.release()
            self.timer.stop()
            self.process_scan()  # Process collected frames
            
    def update_scan(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Add error checking for camera preview
                if self.camera_preview is None:
                    print("Warning: Camera preview not initialized")
                    return
                try:
                    self.camera_preview.update_frame(frame)
                except Exception as e:
                    print(f"Error updating camera preview: {str(e)}")
                    return
                
                # Get camera's field of view (FOV)
                # This should be calibrated for your specific camera
                h_fov = 60  # horizontal FOV in degrees
                v_fov = 45  # vertical FOV in degrees
                
                # Calculate which parts of sphere are covered by this frame
                # This is a simplified example - you should get real orientation from sensors
                yaw = len(self.frames) * (360.0 / 30.0)  # horizontal rotation
                pitch = (len(self.frames) % 4) * (180.0 / 4)  # vertical rotation
                
                self.update_coverage(yaw, pitch, h_fov, v_fov)
                self.frames.append(frame)
                self.frame_positions.append({'yaw': yaw, 'pitch': pitch})
                self.update_visualization(frame)

    def update_coverage(self, yaw, pitch, h_fov, v_fov):
        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)
        h_fov_rad = math.radians(h_fov)
        v_fov_rad = math.radians(v_fov)
        
        # Update coverage for sphere points
        for i, (point, _) in enumerate(self.sphere_points):
            theta, phi = point
            # Check if point is within camera FOV
            angle_diff = abs(theta - yaw_rad)
            if angle_diff <= h_fov_rad/2 and abs(phi - pitch_rad) <= v_fov_rad/2:
                self.sphere_points[i] = (point, True)

    def update_visualization(self, frame):
        if self.vis_widget:
            self.vis_widget.update_coverage(self.sphere_points)

    def process_scan(self):
        if not self.frames:
            return
            
        # Create output directory if it doesn't exist
        import os
        output_dir = "photosphere_output"
        os.makedirs(output_dir, exist_ok=True)
        
        # Save individual frames
        for i, (frame, pos) in enumerate(zip(self.frames, self.frame_positions)):
            cv2.imwrite(f"{output_dir}/frame_{i}.jpg", frame)
            
        try:
            # Convert frames to equirectangular projection
            # This is a placeholder - you'll need a proper stitching algorithm
            equirect = self.stitch_to_equirectangular(self.frames, self.frame_positions)
            
            file_path, _ = QFileDialog.getSaveFileName(
                self, "Save Photosphere", "", "Images (*.jpg)")
            if file_path:
                equirect.save(file_path)
                
        except Exception as e:
            print(f"Error during processing: {str(e)}")

    def stitch_to_equirectangular(self, frames, positions):
        try:
            # This is where you'd implement proper equirectangular stitching
            width, height = 4096, 2048  # Standard equirectangular resolution
            equirect = Image.new('RGB', (width, height), 'gray')
            return equirect
        except Exception as e:
            print(f"Error in stitching: {str(e)}")
            # Return a blank image if stitching fails
            return Image.new('RGB', (1024, 512), 'gray')
        
    def closeEvent(self, event):
        if self.cap:
            self.cap.release()
        if self.vis_widget:
            self.vis_widget.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    scanner = ScannerApp()
    scanner.show()
    sys.exit(app.exec_())
