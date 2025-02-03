import sys
import cv2
import numpy as np
import open3d as o3d
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, 
                           QWidget, QComboBox, QHBoxLayout, QLabel, QFileDialog)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from scipy.spatial.transform import Rotation

class ScannerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.scanning = False
        self.available_cameras = self.get_available_cameras()
        self.setup_ui()
        self.setup_visualizer()
        self.cap = None
        self.frames = []  # Store captured frames
        self.frame_positions = []  # Store orientation data
        self.sphere_vertices = None
        self.coverage_map = None
        self.init_coverage_tracking()

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
        self.setGeometry(100, 100, 800, 600)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Add camera selection controls
        camera_layout = QHBoxLayout()
        camera_label = QLabel("Select Camera:")
        self.camera_combo = QComboBox()
        self.camera_combo.addItems(self.available_cameras)
        camera_layout.addWidget(camera_label)
        camera_layout.addWidget(self.camera_combo)
        layout.addLayout(camera_layout)
        
        self.scan_button = QPushButton('Start Scanning')
        self.scan_button.clicked.connect(self.toggle_scanning)
        layout.addWidget(self.scan_button)
        
    def init_coverage_tracking(self):
        # Create higher resolution sphere for better coverage tracking
        sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=50)
        self.sphere_vertices = np.asarray(sphere_mesh.vertices)
        # Initialize coverage map (0 = not covered, 1 = covered)
        self.coverage_map = np.zeros(len(self.sphere_vertices))
        
    def setup_visualizer(self):
        self.sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=50)
        self.sphere.compute_vertex_normals()
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(width=800, height=600)
        self.vis.add_geometry(self.sphere)
        
        # Set up camera view for better visualization
        ctr = self.vis.get_view_control()
        ctr.set_zoom(0.8)
        
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
        # Convert camera orientation to rotation matrix
        r = Rotation.from_euler('yz', [pitch, yaw], degrees=True)
        camera_direction = r.apply([0, 0, 1])
        
        # Calculate which vertices are in the camera's view
        for i, vertex in enumerate(self.sphere_vertices):
            angle = np.arccos(np.dot(vertex, camera_direction))
            if np.degrees(angle) < max(h_fov, v_fov) / 2:
                self.coverage_map[i] = 1

    def update_visualization(self, frame):
        # Update sphere colors based on coverage
        colors = np.zeros((len(self.sphere_vertices), 3))
        colors[self.coverage_map == 1] = [0, 1, 0]  # Green for covered areas
        colors[self.coverage_map == 0] = [1, 0, 0]  # Red for uncovered areas
        
        self.sphere.vertex_colors = o3d.utility.Vector3dVector(colors)
        self.vis.update_geometry(self.sphere)
        self.vis.poll_events()
        self.vis.update_renderer()
        
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
            from PIL import Image
            equirect = self.stitch_to_equirectangular(self.frames, self.frame_positions)
            
            file_path, _ = QFileDialog.getSaveFileName(
                self, "Save Photosphere", "", "Images (*.jpg)")
            if file_path:
                equirect.save(file_path)
                
        except Exception as e:
            print(f"Error during processing: {str(e)}")

    def stitch_to_equirectangular(self, frames, positions):
        # This is where you'd implement proper equirectangular stitching
        # For now, we'll just create a placeholder image
        width, height = 4096, 2048  # Standard equirectangular resolution
        return Image.new('RGB', (width, height), 'gray')
        
    def closeEvent(self, event):
        if self.cap:
            self.cap.release()
        self.vis.destroy_window()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    scanner = ScannerApp()
    scanner.show()
    sys.exit(app.exec_())
