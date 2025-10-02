"""
PyQT-based GUI for ArduPilot ROS2 interface
"""


from datetime import datetime
from PyQt5.QtWidgets import (QMainWindow, QVBoxLayout, QWidget, QHBoxLayout, 
                           QLabel, QTextEdit, QPushButton, QFrame, QSpinBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont, QTextCursor

from .ros_thread import ROSThread


class ArduPilotGUI(QMainWindow):
    """ArduPilot ROS2 GUI main window class."""

    # Add Qt signals for thread-safe GUI updates
    log_signal = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.ros_thread = None
        self.setWindowTitle("ArduPilot ROS2 GUI")
        self.setGeometry(100, 100, 800, 900)  # Increased height for pre-arm box

        # Connect signals to their handlers (runs in GUI thread)
        self.log_signal.connect(self._handle_log_message)

        # Set up GUI logging callback
        self.node.set_gui_log_callback(self.log_message)

        # Set up velocity monitoring callback for AP velocity only
        self.node.set_ap_velocity_update_callback(self.update_ap_velocity_display)

        # Vehicle type tracking
        self.current_vehicle_type = None

        # Logging control
        self.log_text = None

        # Pre-arm check data
        self.last_prearm_check = None

        # Create central widget - single frame layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout(central_widget)

        # Title
        title_label = QLabel("ArduPilot ROS2 Control Interface")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_style = ("QLabel { padding: 10px; background-color: #2196F3; "
                       "color: white; }")
        title_label.setStyleSheet(title_style)
        main_layout.addWidget(title_label)

        # Create all content directly in main layout
        self.create_main_content(main_layout)

        # Timer for updating GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(200)  # Update every 200ms

        # Timer for pre-arm checks
        self.prearm_timer = QTimer()
        self.prearm_timer.timeout.connect(self.request_prearm_check)
        self.prearm_timer.start(2000)  # Check every 2 seconds

        self.log_message("ArduPilot GUI Started")

        # Start ROS thread
        self.start_ros_thread()

    def create_main_content(self, layout):
        """Create all main content in a single layout."""
        # Status section
        status_layout = QHBoxLayout()
        layout.addLayout(status_layout)

        # Create status labels
        self.gps_label = self._create_status_label("GPS: No Data")
        self.battery_label = self._create_status_label("Battery: No Data")
        self.status_label = self._create_status_label("Vehicle: No Data")
        self.connection_label = self._create_status_label("Status: Connecting...",
                                                          "#ffcccc")

        status_layout.addWidget(self.gps_label)
        status_layout.addWidget(self.battery_label)
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.connection_label)

        # Add Pre-arm Check section
        prearm_frame = self._create_prearm_status()
        layout.addWidget(prearm_frame)

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

        self.arm_button = self._create_button("ARM VEHICLE", "#4CAF50",
                                              self.arm_vehicle)
        self.disarm_button = self._create_button("DISARM VEHICLE", "#f44336",
                                                 self.disarm_vehicle)
        self.debug_button = self._create_button("LIST TOPICS", "#2196F3",
                                                self.list_topics)

        control_layout.addWidget(self.arm_button)
        control_layout.addWidget(self.disarm_button)
        control_layout.addWidget(self.debug_button)

        # Add mode buttons
        mode_frame = self._create_mode_buttons()
        layout.addWidget(mode_frame)

        # Add copter-specific controls
        copter_frame = self._create_copter_controls()
        layout.addWidget(copter_frame)

        # Add velocity control section
        velocity_control = self._create_velocity_control()
        layout.addWidget(velocity_control)

    def _create_prearm_status(self):
        """Create pre-arm check status display."""
        prearm_frame = QFrame()
        prearm_frame.setFrameStyle(QFrame.StyledPanel)
        prearm_frame.setMaximumHeight(80)  # Reduced height since we removed details

        prearm_layout = QVBoxLayout(prearm_frame)

        # Title only (no manual check button)
        prearm_title = QLabel("Pre-Arm Check Status:")
        prearm_title.setFont(QFont("Arial", 12, QFont.Bold))
        prearm_layout.addWidget(prearm_title)

        # Status display
        self.prearm_status_label = QLabel("Status: Checking...")
        self.prearm_status_label.setFont(QFont("Arial", 10, QFont.Bold))
        prearm_style = ("QLabel { background-color: #fff3cd; padding: 8px; "
                        "border: 2px solid #ffc107; border-radius: 3px; }")
        self.prearm_status_label.setStyleSheet(prearm_style)
        prearm_layout.addWidget(self.prearm_status_label)

        return prearm_frame

    def request_prearm_check(self):
        """Request pre-arm check from ArduPilot, if vehicle detected."""
        data_received = any([
            self.node.gps_data,
            self.node.battery_data,
            self.node.status_data
        ])
        if data_received:
            self.node.prearm_check()

    def update_prearm_display(self):
        """Update the pre-arm status display."""
        try:
            # Check if we have recent data (within last 10 seconds)
            current_time = datetime.now()
            if (self.last_prearm_check and 
                (current_time - self.last_prearm_check).total_seconds() > 10):
                self.prearm_status_label.setText("Status: Data outdated")
                outdated_style = ("QLabel { background-color: #f8d7da; padding: 8px; "
                                  "border: 2px solid #dc3545; border-radius: 3px; }")
                self.prearm_status_label.setStyleSheet(outdated_style)
                return

            # Display current status
            can_arm = self.node.pre_arm_status

            if can_arm:
                self.prearm_status_label.setText("Status: READY TO ARM")
                ready_style = ("QLabel { background-color: #d1e7dd; padding: 8px; "
                               "border: 2px solid #198754; border-radius: 3px; "
                               "color: #198754; }")
                self.prearm_status_label.setStyleSheet(ready_style)
            else:
                self.prearm_status_label.setText("Status: NOT READY TO ARM")
                not_ready_style = ("QLabel { background-color: #f8d7da; padding: 8px; "
                                   "border: 2px solid #dc3545; border-radius: 3px; "
                                   "color: #dc3545; }")
                self.prearm_status_label.setStyleSheet(not_ready_style)
            
        except Exception as e:
            self.log_message(f"Error updating pre-arm display: {e}")
            self.prearm_status_label.setText("Status: Display Error")
            error_style = ("QLabel { background-color: #f8d7da; padding: 8px; "
                           "border: 2px solid #dc3545; border-radius: 3px; }")
            self.prearm_status_label.setStyleSheet(error_style)

    def start_ros_thread(self):
        """Initialize and start the ROS thread."""
        self.ros_thread = ROSThread(self.node)
        self.ros_thread.start()

    def _create_status_label(self, text, background_color="#f0f0f0"):
        """Create a status label with styling."""
        label = QLabel(text)
        label.setFont(QFont("Arial", 10))
        label.setAlignment(Qt.AlignCenter)
        label_style = (f"QLabel {{ background-color: {background_color}; "
                       f"padding: 10px; border: 2px solid #ccc; margin: 5px; "
                       f"min-height: 60px; }}")
        label.setStyleSheet(label_style)
        label.setWordWrap(True)
        return label

    def _create_log_text(self):
        """Create the log text widget."""
        log_text = QTextEdit()
        log_text.setFont(QFont("Courier", 9))
        log_text.setMaximumHeight(200)
        log_style = ("QTextEdit { background-color: #f5f5f5; "
                     "border: 2px solid #ccc; }")
        log_text.setStyleSheet(log_style)
        log_text.setReadOnly(True)
        return log_text

    def _create_button(self, text, color, callback):
        """Create a styled button."""
        button = QPushButton(text)
        button.setFont(QFont("Arial", 11, QFont.Bold))
        button_style = (f"QPushButton {{ background-color: {color}; "
                        f"color: white; padding: 10px; border: none; "
                        f"border-radius: 5px; margin: 5px; }} "
                        f"QPushButton:hover {{ opacity: 0.8; }} "
                        f"QPushButton:pressed {{ background-color: #333; }} "
                        f"QPushButton:disabled {{ background-color: #cccccc; "
                        f"color: #666666; }}")
        button.setStyleSheet(button_style)
        button.setMinimumHeight(40)
        button.clicked.connect(callback)
        return button

    def _create_mode_button(self, text, callback):
        """Create a mode button with consistent styling."""
        button = QPushButton(text)
        button.setFont(QFont("Arial", 10, QFont.Bold))
        btn_style = ("QPushButton { background-color: #2196F3; color: white; "
                     "padding: 8px; border-radius: 3px; margin: 2px; "
                     "min-width: 70px; } "
                     "QPushButton:disabled { background-color: #cccccc; "
                     "color: #666666; }")
        button.setStyleSheet(btn_style)
        button.clicked.connect(callback)
        return button

    def update_gui(self):
        """Update GUI with latest data from ROS node."""
        try:
            self._update_gps_display()
            self._update_battery_display()
            self._update_status_display()
            self._update_connection_status()
            self._update_button_states()
            self.update_prearm_display()
        except Exception as e:
            self.log_message(f"GUI update error: {e}")

    def _update_gps_display(self):
        """Update GPS display."""
        if self.node.gps_data:
            lat = self.node.gps_data['lat']
            lon = self.node.gps_data['lon']
            alt = self.node.gps_data['alt']
            self.gps_label.setText(f"GPS: {lat:.6f}, {lon:.6f}\nAlt: {alt:.1f}m")
            gps_style = ("QLabel { background-color: #ccffcc; padding: 10px; "
                         "border: 2px solid #4CAF50; margin: 5px; "
                         "min-height: 60px; }")
            self.gps_label.setStyleSheet(gps_style)
        else:
            self.gps_label.setText("GPS: No Data")
            no_gps_style = ("QLabel { background-color: #f0f0f0; padding: 10px; "
                            "border: 2px solid #ccc; margin: 5px; "
                            "min-height: 60px; }")
            self.gps_label.setStyleSheet(no_gps_style)

    def _update_battery_display(self):
        """Update battery display."""
        if self.node.battery_data:
            voltage = self.node.battery_data['voltage']
            percentage = self.node.battery_data['percentage']
            current = self.node.battery_data['current']
            self.battery_label.setText(f"Battery: {voltage:.2f}V\n"
                                       f"{current:.1f}A\n{percentage:.0f}%")

            # Color code battery status
            if percentage > 50:
                color, border = "#ccffcc", "#4CAF50"
            elif percentage > 20:
                color, border = "#ffffcc", "#FF9800"
            else:
                color, border = "#ffcccc", "#f44336"

            battery_style = (f"QLabel {{ background-color: {color}; "
                            f"padding: 10px; border: 2px solid {border}; "
                            f"margin: 5px; min-height: 60px; }}")
            self.battery_label.setStyleSheet(battery_style)
        else:
            self.battery_label.setText("Battery: No Data")
            no_battery_style = ("QLabel { background-color: #f0f0f0; padding: 10px; "
                                "border: 2px solid #ccc; margin: 5px; "
                                "min-height: 60px; }")
            self.battery_label.setStyleSheet(no_battery_style)

    def _update_button_states(self):
        """Enable/disable buttons based on vehicle status and type availability."""

        # Update ARM/DISARM buttons
        self.arm_button.setEnabled(self.node.vehicle_active)
        self.disarm_button.setEnabled(self.node.vehicle_active)

        self._update_vehicle_specific_controls()

        # Update mode buttons based on vehicle type and status
        self._update_mode_button_availability(self.node.vehicle_active)

    def _update_vehicle_specific_controls(self):
        """Enable/disable vehicle-specific controls based on vehicle type."""
        is_copter = self.node.get_vehicle_type() == "COPTER"

        # Enable/disable copter controls instead of hiding them
        self.copter_frame.setEnabled(is_copter)

        if is_copter:
            self.copter_frame.setStyleSheet("")
        else:
            self.copter_frame.setStyleSheet("QFrame { color: #999999; }")

    def _update_mode_button_availability(self, status_available):
        """Enable/disable mode buttons based on vehicle type and status."""
        # Common modes available to all vehicles when status is available
        common_buttons = [self.mode_guided_btn, self.mode_auto_btn, 
                          self.mode_rtl_btn, self.mode_loiter_btn]

        for button in common_buttons:
            button.setEnabled(status_available)

        # Vehicle specific modes and functions
        if self.node.get_vehicle_type() == "ROVER":
            # Rover only
            self.mode_hold_btn.setEnabled(status_available)
            self.mode_manual_btn.setEnabled(status_available)
        else:
            self.mode_hold_btn.setEnabled(False)
            self.mode_manual_btn.setEnabled(False)

        if self.node.get_vehicle_type() == "COPTER":
            # Copter only
            self.mode_land_btn.setEnabled(status_available)
            self.takeoff_button.setEnabled(status_available)
        else:
            self.mode_land_btn.setEnabled(False)
            self.takeoff_button.setEnabled(False)

    def set_mode(self, mode):
        """Send mode change command to vehicle."""
        try:
            # Check if vehicle status is available
            if not self.node.vehicle_active:
                self.log_message("Cannot change mode: Vehicle status not available")
                return

            self.node.mode_switch(mode)

            # Update button colors to reflect the requested mode
            self._update_mode_buttons(mode)

        except Exception as e:
            self.log_message(f"Mode change error: {e}")

    def takeoff_to_altitude(self):
        """Send takeoff command to copter."""
        try:
            if not self.node.status_data:
                self.log_message("Cannot takeoff: Vehicle status not available")
                return

            altitude = self.takeoff_altitude_spin.value()
            self.log_message(f"Takeoff to {altitude}m requested")
            # Fix: Call the correct method name
            self.node.takeoff_copter(altitude)

        except Exception as e:
            self.log_message(f"Takeoff command error: {e}")

    def _update_mode_buttons(self, mode):
        """Set colors for mode buttons based on current mode."""
        modes = ['HOLD', 'GUIDED', 'AUTO', 'MANUAL', 'RTL', 'LOITER', 'LAND']
        btn_normal = ("QPushButton { background-color: #2196F3; color: white; "
                      "padding: 8px; border-radius: 3px; margin: 2px; "
                      "min-width: 70px; } "
                      "QPushButton:disabled { background-color: #cccccc; "
                      "color: #666666; }")
        btn_active = ("QPushButton { background-color: #4CAF50; color: white; "
                      "padding: 8px; border-radius: 3px; margin: 2px; "
                      "min-width: 70px; } "
                      "QPushButton:disabled { background-color: #cccccc; "
                      "color: #666666; }")

        # Update button colors based on current mode
        for mode_name in modes:
            btn = getattr(self, f'mode_{mode_name.lower()}_btn', None)
            if btn:
                if mode.upper() == mode_name.upper():
                    btn.setStyleSheet(btn_active)
                else:
                    btn.setStyleSheet(btn_normal)

    def _create_mode_buttons(self):
        """Create flight mode control buttons."""
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
        self.mode_hold_btn = self._create_mode_button("HOLD",
                                                      lambda: self.set_mode("HOLD"))
        self.mode_guided_btn = self._create_mode_button("GUIDED",
                                                        lambda: self.set_mode("GUIDED"))
        self.mode_auto_btn = self._create_mode_button("AUTO",
                                                      lambda: self.set_mode("AUTO"))
        self.mode_manual_btn = self._create_mode_button("MANUAL",
                                                        lambda: self.set_mode("MANUAL"))
        self.mode_rtl_btn = self._create_mode_button("RTL",
                                                     lambda: self.set_mode("RTL"))
        self.mode_loiter_btn = self._create_mode_button("LOITER",
                                                        lambda: self.set_mode("LOITER"))

        # Add buttons to horizontal layout
        mode_layout.addWidget(self.mode_hold_btn)
        mode_layout.addWidget(self.mode_guided_btn)
        mode_layout.addWidget(self.mode_auto_btn)
        mode_layout.addWidget(self.mode_manual_btn)
        mode_layout.addWidget(self.mode_rtl_btn)
        mode_layout.addWidget(self.mode_loiter_btn)

        mode_main_layout.addLayout(mode_layout)

        return mode_frame

    def _create_copter_controls(self):
        """Create copter-specific controls (takeoff, land)."""
        self.copter_frame = QFrame()
        self.copter_frame.setFrameStyle(QFrame.StyledPanel)
        self.copter_frame.setMaximumHeight(100)

        copter_main_layout = QVBoxLayout(self.copter_frame)

        # Title
        copter_title = QLabel("Copter Controls:")
        copter_title.setFont(QFont("Arial", 12, QFont.Bold))
        copter_main_layout.addWidget(copter_title)

        # Controls layout
        controls_layout = QHBoxLayout()

        # Takeoff controls
        takeoff_layout = QHBoxLayout()

        takeoff_label = QLabel("Takeoff to:")
        takeoff_label.setFont(QFont("Arial", 10))
        takeoff_layout.addWidget(takeoff_label)

        self.takeoff_altitude_spin = QSpinBox()
        self.takeoff_altitude_spin.setRange(1, 100)
        self.takeoff_altitude_spin.setValue(10)
        self.takeoff_altitude_spin.setSuffix(" m")
        self.takeoff_altitude_spin.setFont(QFont("Arial", 10))
        takeoff_layout.addWidget(self.takeoff_altitude_spin)

        self.takeoff_button = QPushButton("TAKEOFF")
        self.takeoff_button.setFont(QFont("Arial", 10, QFont.Bold))
        takeoff_style = ("QPushButton { background-color: #FF9800; color: white; "
                         "padding: 8px; border-radius: 3px; margin: 2px; } "
                         "QPushButton:disabled { background-color: #cccccc; "
                         "color: #666666; }")
        self.takeoff_button.setStyleSheet(takeoff_style)
        self.takeoff_button.clicked.connect(self.takeoff_to_altitude)
        takeoff_layout.addWidget(self.takeoff_button)

        controls_layout.addLayout(takeoff_layout)

        # Land button
        self.mode_land_btn = self._create_mode_button("LAND", 
                                                      lambda: self.set_mode("LAND"))
        land_style = ("QPushButton { background-color: #FF5722; color: white; "
                      "padding: 8px; border-radius: 3px; margin: 2px; "
                      "min-width: 70px; } "
                      "QPushButton:disabled { background-color: #cccccc; "
                      "color: #666666; }")
        self.mode_land_btn.setStyleSheet(land_style)
        controls_layout.addWidget(self.mode_land_btn)

        copter_main_layout.addLayout(controls_layout)

        return self.copter_frame

    def _create_velocity_control(self):
        """Create velocity information display (read-only) for /ap/cmd_vel only."""
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
        self.current_ap_vel_label = QLabel("Current: Linear(0.0,0.0,0.0)m/s "
                                           "Angular(0.0,0.0,0.0)rad/sec")
        self.current_ap_vel_label.setFont(QFont("Courier", 9))
        vel_label_style = ("QLabel { background-color: #e3f2fd; padding: 4px; "
                           "border: 1px solid #1976D2; }")
        self.current_ap_vel_label.setStyleSheet(vel_label_style)
        self.current_ap_vel_label.setWordWrap(True)
        vel_layout.addWidget(self.current_ap_vel_label)

        # Initialize velocity tracking for AP topic only
        self.last_ap_velocity_time = None
        self.current_ap_velocity = {'x': 0, 'y': 0, 'z': 0,
                                    'roll': 0, 'pitch': 0, 'yaw': 0}

        return vel_frame

    def update_ap_velocity_display(self):
        """Update the /ap/cmd_vel display with received data (simplified)."""
        try:
            vel = self.node.ap_velocity_data

            # Update main display only
            display_text = (f"Current: Linear({vel['x']:.2f},{vel['y']:.2f},"
                            f"{vel['z']:.2f})m/s Angular({vel['roll']:.2f},"
                            f"{vel['pitch']:.2f},{vel['yaw']:.2f})rad/sec")
            self.current_ap_vel_label.setText(display_text)

        except Exception as e:
            self.log_message(f"AP velocity display update error: {e}")

    def _update_status_display(self):
        """Update vehicle status display."""
        if self.node.status_data != {}:
            armed = self.node.status_data.get('armed', False)
            mode = self.node.status_data.get('mode', 'UNKNOWN')
            vehicle_type = self.node.status_data.get('vehicle_type', 'Unknown')
            failsafe = self.node.status_data.get('failsafe', 'None')

            armed_text = "ARMED" if armed else "DISARMED"
            self.status_label.setText(f"{vehicle_type}: {armed_text}\n"
                                      f"Mode: {mode}\nFailsafe: {failsafe}")

            # Update mode buttons to reflect current mode
            self._update_mode_buttons(mode)

            # Color code based on armed status
            if armed:
                color, border = "#ffcccc", "#f44336"
            else:
                color, border = "#ccffcc", "#4CAF50"

            status_style = (f"QLabel {{ background-color: {color}; "
                            f"padding: 10px; border: 2px solid {border}; "
                            f"margin: 5px; min-height: 60px; }}")
            self.status_label.setStyleSheet(status_style)
        else:
            self.status_label.setText("Vehicle: No Data")
            no_status_style = ("QLabel { background-color: #f0f0f0; padding: 10px; "
                               "border: 2px solid #ccc; margin: 5px; "
                               "min-height: 60px; }")
            self.status_label.setStyleSheet(no_status_style)

    def _update_connection_status(self):
        """Update connection status."""
        data_received = any([
            self.node.gps_data,
            self.node.battery_data,
            self.node.status_data
        ])

        if data_received:
            self.connection_label.setText("DDS Status: Connected\nReceiving Data")
            connected_style = ("QLabel { background-color: #ccffcc; "
                               "padding: 10px; border: 2px solid #4CAF50; "
                               "margin: 5px; min-height: 60px; }")
            self.connection_label.setStyleSheet(connected_style)
        else:
            self.connection_label.setText("DDS Status: Connected\nNo Data")
            no_data_style = ("QLabel { background-color: #f0f0f0; "
                             "padding: 10px; border: 2px solid #ccc; "
                             "margin: 5px; min-height: 60px; }")
            self.connection_label.setStyleSheet(no_data_style)

    def log_message(self, message):
        """Thread-safe log message method using Qt signals."""
        # Emit signal instead of directly manipulating GUI
        self.log_signal.emit(message)

    def _handle_log_message(self, message):
        """Thread-safe implementation of log message - runs in GUI thread."""
        try:
            timestamp = self.get_timestamp()
            formatted_message = f"[{timestamp}] {message}"

            if hasattr(self, 'log_text') and self.log_text is not None:
                # Safe GUI manipulation - we're in the GUI thread
                self.log_text.append(formatted_message)

                # Keep only last 100 lines - safer implementation
                document = self.log_text.document()
                if document.blockCount() > 100:
                    cursor = QTextCursor(document)
                    cursor.movePosition(QTextCursor.Start)
                    cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor)
                    cursor.removeSelectedText()

                # Auto-scroll to bottom - safer implementation
                scrollbar = self.log_text.verticalScrollBar()
                scrollbar.setValue(scrollbar.maximum())
            else:
                # Fallback to console if log_text not ready
                print(formatted_message)

        except Exception as e:
            print(f"Logging error: {e}")

    def get_timestamp(self):
        """Get current timestamp string."""
        return datetime.now().strftime("%H:%M:%S")

    def list_topics(self):
        """List all available ROS topics for debugging."""
        try:
            topic_names_and_types = self.node.get_topic_names_and_types()
            self.log_message("=== All ROS Topics ===")

            for name, msg_types in topic_names_and_types:
                self.log_message(f"{name} ({', '.join(msg_types)})")

            self.log_message("=== End Topic List ===")

        except Exception as e:
            self.log_message(f"Error listing topics: {e}")

    def arm_vehicle(self):
        """Send ARM command to vehicle."""
        try:
            # Check if vehicle status is available
            if not self.node.vehicle_active:
                self.log_message("Cannot arm: Vehicle status not available")
                return

            self.log_message("ARM command sent")
            self.node.arm_vehicle()
        except Exception as e:
            self.log_message(f"ARM command error: {e}")

    def disarm_vehicle(self):
        """Send DISARM command to vehicle."""
        try:
            # Check if vehicle status is available
            if not self.node.vehicle_active:
                self.log_message("Cannot disarm: Vehicle status not available")
                return

            self.log_message("DISARM command sent")
            self.node.disarm_vehicle()
        except Exception as e:
            self.log_message(f"DISARM command error: {e}")

    def closeEvent(self, event):
        """Handle window close event."""
        self.log_message("Shutting down GUI...")
        try:
            if self.ros_thread:
                self.ros_thread.stop()
                self.ros_thread.wait(1000)  # Wait up to 1 second
        except Exception as e:
            print(f"Error stopping ROS thread: {e}")

        event.accept()
