#!/usr/bin/env python3

from datetime import datetime
from PyQt5.QtWidgets import (QMainWindow, QVBoxLayout, QWidget, QHBoxLayout, 
                           QLabel, QTextEdit, QPushButton, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

from .ros_thread import ROSThread


class ArduPilotGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.ros_thread = None
        self.setWindowTitle("ArduPilot ROS2 GUI")
        self.setGeometry(100, 100, 800, 800)

        # Set up GUI logging callback
        self.node.set_gui_log_callback(self.log_message)
        
        # Set up velocity monitoring callback for AP velocity only
        self.node.set_ap_velocity_update_callback(self.update_ap_velocity_display)

        # Create central widget - single frame layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout(central_widget)

        # Title
        title_label = QLabel("ArduPilot ROS2 Control Interface")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setStyleSheet("QLabel { padding: 10px; background-color: #2196F3; color: white; }")
        main_layout.addWidget(title_label)

        # Create all content directly in main layout
        self.create_main_content(main_layout)

        # Timer for updating GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(200)  # Update every 200ms

        self.log_message("ArduPilot GUI Started")
        self.log_message("Waiting for ArduPilot connection...")

        # Start ROS thread
        self.start_ros_thread()

    def create_main_content(self, layout):
        """Create all main content in a single layout"""
        # Status section
        status_layout = QHBoxLayout()
        layout.addLayout(status_layout)

        # Create status labels
        self.gps_label = self._create_status_label("GPS: No Data")
        self.battery_label = self._create_status_label("Battery: No Data")
        self.status_label = self._create_status_label("Vehicle: No Data")
        self.connection_label = self._create_status_label("Status: Connecting...", "#ffcccc")

        status_layout.addWidget(self.gps_label)
        status_layout.addWidget(self.battery_label)
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.connection_label)

        # Message log
        log_label = QLabel("Message Log:")
        log_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(log_label)

        self.log_text = self._create_log_text()
        layout.addWidget(self.log_text)

        # Control buttons
        control_label = QLabel("Vehicle Control:")
        control_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(control_label)

        control_layout = QHBoxLayout()
        layout.addLayout(control_layout)

        self.arm_button = self._create_button("ARM VEHICLE", "#4CAF50", self.arm_vehicle)
        self.disarm_button = self._create_button("DISARM VEHICLE", "#f44336", self.disarm_vehicle)
        self.debug_button = self._create_button("LIST TOPICS", "#2196F3", self.list_topics)

        control_layout.addWidget(self.arm_button)
        control_layout.addWidget(self.disarm_button)
        control_layout.addWidget(self.debug_button)

        # Add mode buttons
        mode_frame = self._create_mode_buttons()
        layout.addWidget(mode_frame)
        
        # Add velocity control section
        velocity_control = self._create_velocity_control()
        layout.addWidget(velocity_control)

    def start_ros_thread(self):
        """Initialize and start the ROS thread"""
        self.ros_thread = ROSThread(self.node)
        self.ros_thread.start()

    def _create_status_label(self, text, background_color="#f0f0f0"):
        """Create a status label with styling"""
        label = QLabel(text)
        label.setFont(QFont("Arial", 10))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f"QLabel {{ background-color: {background_color}; "
                           f"padding: 10px; border: 2px solid #ccc; margin: 5px; "
                           f"min-height: 60px; }}")
        label.setWordWrap(True)
        return label

    def _create_log_text(self):
        """Create the log text widget"""
        log_text = QTextEdit()
        log_text.setFont(QFont("Courier", 9))
        log_text.setMaximumHeight(200)
        log_text.setStyleSheet("QTextEdit { background-color: #f5f5f5; "
                              "border: 2px solid #ccc; }")
        log_text.setReadOnly(True)
        return log_text

    def _create_button(self, text, color, callback):
        """Create a styled button"""
        button = QPushButton(text)
        button.setFont(QFont("Arial", 11, QFont.Bold))
        button.setStyleSheet(f"QPushButton {{ background-color: {color}; "
                           f"color: white; padding: 10px; border: none; "
                           f"border-radius: 5px; margin: 5px; }} "
                           f"QPushButton:hover {{ opacity: 0.8; }} "
                           f"QPushButton:pressed {{ background-color: #333; }}")
        button.setMinimumHeight(40)
        button.clicked.connect(callback)
        return button

    def update_gui(self):
        """Update GUI with latest data from ROS node"""
        try:
            self._update_gps_display()
            self._update_battery_display()
            self._update_status_display()
            self._update_connection_status()
        except Exception as e:
            self.log_message(f"GUI update error: {e}")

    def _update_gps_display(self):
        """Update GPS display"""
        if hasattr(self.node, 'gps_data') and self.node.gps_data:
            lat = self.node.gps_data['lat']
            lon = self.node.gps_data['lon']
            alt = self.node.gps_data['alt']
            self.gps_label.setText(f"GPS: {lat:.6f}, {lon:.6f}\nAlt: {alt:.1f}m")
            self.gps_label.setStyleSheet("QLabel { background-color: #ccffcc; padding: 10px; "
                                       "border: 2px solid #4CAF50; margin: 5px; min-height: 60px; }")
        else:
            self.gps_label.setText("GPS: No Data")
            self.gps_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 10px; "
                                       "border: 2px solid #ccc; margin: 5px; min-height: 60px; }")

    def _update_battery_display(self):
        """Update battery display"""
        if hasattr(self.node, 'battery_data') and self.node.battery_data:
            voltage = self.node.battery_data['voltage']
            percentage = self.node.battery_data['percentage']
            current = abs(self.node.battery_data['current'])
            self.battery_label.setText(f"Battery: {voltage:.2f}V\n{current:.1f}A\n{percentage:.0f}%")

            # Color code battery status
            if percentage > 50:
                color, border = "#ccffcc", "#4CAF50"
            elif percentage > 20:
                color, border = "#ffffcc", "#FF9800"
            else:
                color, border = "#ffcccc", "#f44336"
            
            self.battery_label.setStyleSheet(f"QLabel {{ background-color: {color}; "
                                           f"padding: 10px; border: 2px solid {border}; margin: 5px; min-height: 60px; }}")
        else:
            self.battery_label.setText("Battery: No Data")
            self.battery_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 10px; "
                                            "border: 2px solid #ccc; margin: 5px; min-height: 60px; }")

    def set_mode(self, mode):
        """Send mode change command to vehicle"""
        try:
            self.log_message(f"Mode change requested: {mode}")
            self.node.mode_switch(mode)
            
            # Update button colors to reflect the requested mode
            self._update_mode_buttons(mode)
            
        except Exception as e:
            self.log_message(f"Mode change error: {e}")

    def _update_mode_buttons(self, mode):
        """Set colors for mode buttons based on current mode"""
        modes = ['HOLD', 'GUIDED', 'AUTO', 'MANUAL', 'RTL']
        btn_normal = "QPushButton { background-color: #2196F3; color: white; padding: 8px; border-radius: 3px; margin: 2px; }"
        btn_active = "QPushButton { background-color: #4CAF50; color: white; padding: 8px; border-radius: 3px; margin: 2px; }"
        
        # Update button colors based on current mode
        for mode_name in modes:
            btn = getattr(self, f'mode_{mode_name.lower()}_btn', None)
            if btn:
                if mode.upper() == mode_name.upper():
                    btn.setStyleSheet(btn_active)
                else:
                    btn.setStyleSheet(btn_normal)

    def _create_mode_buttons(self):
        """Create flight mode control buttons"""
        # Create a frame for mode buttons
        mode_frame = QFrame()
        mode_frame.setFrameStyle(QFrame.StyledPanel)
        mode_frame.setMaximumHeight(80)
        
        mode_main_layout = QVBoxLayout(mode_frame)
        
        # Title
        mode_title = QLabel("Flight Mode Control:")
        mode_title.setFont(QFont("Arial", 12, QFont.Bold))
        mode_main_layout.addWidget(mode_title)
        
        # Button layout - horizontal arrangement
        mode_layout = QHBoxLayout()
        
        # Create buttons for each mode
        btn_style = "QPushButton { background-color: #2196F3; color: white; padding: 8px; border-radius: 3px; margin: 2px; }"
        
        self.mode_hold_btn = QPushButton("HOLD")        
        self.mode_hold_btn.setStyleSheet(btn_style)
        self.mode_guided_btn = QPushButton("GUIDED")
        self.mode_guided_btn.setStyleSheet(btn_style)
        self.mode_auto_btn = QPushButton("AUTO")
        self.mode_auto_btn.setStyleSheet(btn_style)
        self.mode_manual_btn = QPushButton("MANUAL")
        self.mode_manual_btn.setStyleSheet(btn_style)
        self.mode_rtl_btn = QPushButton("RTL")
        self.mode_rtl_btn.setStyleSheet(btn_style)

        # Connect buttons to mode change function
        self.mode_hold_btn.clicked.connect(lambda: self.set_mode("HOLD"))
        self.mode_guided_btn.clicked.connect(lambda: self.set_mode("GUIDED"))
        self.mode_auto_btn.clicked.connect(lambda: self.set_mode("AUTO"))
        self.mode_manual_btn.clicked.connect(lambda: self.set_mode("MANUAL"))
        self.mode_rtl_btn.clicked.connect(lambda: self.set_mode("RTL"))

        # Add buttons to horizontal layout
        mode_layout.addWidget(self.mode_hold_btn)
        mode_layout.addWidget(self.mode_guided_btn)
        mode_layout.addWidget(self.mode_auto_btn)
        mode_layout.addWidget(self.mode_manual_btn)
        mode_layout.addWidget(self.mode_rtl_btn)
        
        mode_main_layout.addLayout(mode_layout)
        
        return mode_frame

    def _create_velocity_control(self):
        """Create velocity information display (read-only) for /ap/cmd_vel only"""
        vel_frame = QFrame()
        vel_frame.setFrameStyle(QFrame.StyledPanel)
        vel_frame.setMaximumHeight(160)
        
        vel_layout = QVBoxLayout(vel_frame)
        
        # Title
        vel_title = QLabel("Commanded Velocity:")
        vel_title.setFont(QFont("Arial", 12, QFont.Bold))
        vel_layout.addWidget(vel_title)
        
        # Topic title
        ap_title = QLabel("/ap/cmd_vel")
        ap_title.setFont(QFont("Arial", 10, QFont.Bold))
        ap_title.setStyleSheet("QLabel { color: #1976D2; }")
        vel_layout.addWidget(ap_title)
        
        # Current velocity display
        self.current_ap_vel_label = QLabel("Current: Linear(0.0,0.0,0.0) Angular(0.0,0.0,0.0)")
        self.current_ap_vel_label.setFont(QFont("Courier", 9))
        self.current_ap_vel_label.setStyleSheet("QLabel { background-color: #e3f2fd; padding: 4px; border: 1px solid #1976D2; }")
        self.current_ap_vel_label.setWordWrap(True)
        vel_layout.addWidget(self.current_ap_vel_label)
        
        # Status display
        self.ap_vel_status_label = QLabel("Status: No velocity commands received")
        self.ap_vel_status_label.setFont(QFont("Arial", 8))
        self.ap_vel_status_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 3px; }")
        vel_layout.addWidget(self.ap_vel_status_label)
        
        # Initialize velocity tracking for AP topic only
        self.last_ap_velocity_time = None
        self.current_ap_velocity = {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0}
        
        return vel_frame

    def update_ap_velocity_display(self, twist_msg=None):
        """Update the /ap/cmd_vel display with received data (simplified)"""
        try:
            if twist_msg:
                # Update from received Twist message
                self.current_ap_velocity = {
                    'x': twist_msg.linear.x,
                    'y': twist_msg.linear.y,
                    'z': twist_msg.linear.z,
                    'roll': twist_msg.angular.x,
                    'pitch': twist_msg.angular.y,
                    'yaw': twist_msg.angular.z
                }
                self.last_ap_velocity_time = self.get_timestamp()
            
            vel = self.current_ap_velocity
            
            # Update main display only
            display_text = f"Current: Linear({vel['x']:.2f},{vel['y']:.2f},{vel['z']:.2f}) Angular({vel['roll']:.2f},{vel['pitch']:.2f},{vel['yaw']:.2f})"
            self.current_ap_vel_label.setText(display_text)
            
            # Update status
            if twist_msg:
                # Check if any velocity is non-zero
                any_motion = any(abs(v) > 0.01 for v in vel.values())
                if any_motion:
                    self.ap_vel_status_label.setText(f"Status: Active command ({self.last_ap_velocity_time})")
                    self.ap_vel_status_label.setStyleSheet("QLabel { background-color: #c8e6c9; padding: 3px; color: #2E7D32; }")
                else:
                    self.ap_vel_status_label.setText(f"Status: Zero command ({self.last_ap_velocity_time})")
                    self.ap_vel_status_label.setStyleSheet("QLabel { background-color: #fff3cd; padding: 3px; color: #856404; }")
            else:
                self.ap_vel_status_label.setText("Status: No commands received")
                self.ap_vel_status_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 3px; color: #6c757d; }")
                
        except Exception as e:
            self.log_message(f"AP velocity display update error: {e}")

    def _update_status_display(self):
        """Update vehicle status display"""
        if hasattr(self.node, 'status_data') and self.node.status_data:
            armed = self.node.status_data.get('armed', False)
            mode = self.node.status_data.get('mode', 'UNKNOWN')
            vehicle_type = self.node.status_data.get('vehicle_type', 'Unknown')
            failsafe = self.node.status_data.get('failsafe', 'None')

            armed_text = "ARMED" if armed else "DISARMED"
            self.status_label.setText(f"{vehicle_type}: {armed_text}\nMode: {mode}\nFailsafe: {failsafe}")

            # Update mode buttons to reflect current mode
            self._update_mode_buttons(mode)

            # Color code based on armed status
            if armed:
                color, border = "#ffcccc", "#f44336"
            else:
                color, border = "#ccffcc", "#4CAF50"
            
            self.status_label.setStyleSheet(f"QLabel {{ background-color: {color}; "
                                          f"padding: 10px; border: 2px solid {border}; margin: 5px; min-height: 60px; }}")
        else:
            self.status_label.setText("Vehicle: No Data")
            self.status_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 10px; "
                                          "border: 2px solid #ccc; margin: 5px; min-height: 60px; }")

    def _update_connection_status(self):
        """Update connection status"""
        data_received = any([
            hasattr(self.node, 'gps_data') and self.node.gps_data,
            hasattr(self.node, 'battery_data') and self.node.battery_data,
            hasattr(self.node, 'status_data') and self.node.status_data
        ])

        if data_received:
            self.connection_label.setText("DDS Status: Connected\nReceiving Data")
            self.connection_label.setStyleSheet("QLabel { background-color: #ccffcc; "
                                               "padding: 10px; border: 2px solid #4CAF50; margin: 5px; min-height: 60px; }")
        else:
            self.connection_label.setText("DDS Status: Connected\nNo Data")
            self.connection_label.setStyleSheet("QLabel { background-color: #ffffcc; "
                                               "padding: 10px; border: 2px solid #FF9800; margin: 5px; min-height: 60px; }")

    def log_message(self, message):
        """Log a message to the GUI"""
        try:
            timestamp = self.get_timestamp()
            formatted_message = f"[{timestamp}] {message}"
            
            if hasattr(self, 'log_text'):
                self.log_text.append(formatted_message)
                
                # Keep only last 100 lines
                if self.log_text.document().blockCount() > 100:
                    cursor = self.log_text.textCursor()
                    cursor.movePosition(cursor.Start)
                    cursor.select(cursor.BlockUnderCursor)
                    cursor.removeSelectedText()
                
                # Auto-scroll to bottom
                self.log_text.moveCursor(self.log_text.textCursor().End)
            else:
                # Fallback to console if log_text not ready
                print(formatted_message)
                
        except Exception as e:
            print(f"Logging error: {e}")

    def get_timestamp(self):
        """Get current timestamp string"""
        return datetime.now().strftime("%H:%M:%S")

    def list_topics(self):
        """List all available ROS topics for debugging"""
        try:
            topic_names_and_types = self.node.get_topic_names_and_types()
            self.log_message("=== All ROS Topics ===")
            
            for name, msg_types in topic_names_and_types:
                self.log_message(f"{name} ({', '.join(msg_types)})")
                
            self.log_message("=== End Topic List ===")
            
        except Exception as e:
            self.log_message(f"Error listing topics: {e}")

    def arm_vehicle(self):
        """Send ARM command to vehicle"""
        try:
            self.log_message("ARM command sent")
            self.node.arm_vehicle()
        except Exception as e:
            self.log_message(f"ARM command error: {e}")

    def disarm_vehicle(self):
        """Send DISARM command to vehicle"""
        try:
            self.log_message("DISARM command sent")
            self.node.disarm_vehicle()
        except Exception as e:
            self.log_message(f"DISARM command error: {e}")

    def closeEvent(self, event):
        """Handle window close event"""
        self.log_message("Shutting down GUI...")
        try:
            if self.ros_thread:
                self.ros_thread.stop()
                self.ros_thread.wait(1000)  # Wait up to 1 second
        except Exception as e:
            print(f"Error stopping ROS thread: {e}")
        
        event.accept()
