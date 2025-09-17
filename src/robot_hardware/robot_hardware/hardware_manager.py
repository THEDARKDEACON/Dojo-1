#!/usr/bin/env python3
"""
Hardware Manager for Dojo Robot
Coordinates all hardware drivers and provides unified interface
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time
from enum import Enum

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, Range
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class HardwareState(Enum):
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    ERROR = "ERROR"
    SHUTDOWN = "SHUTDOWN"

class HardwareManager(Node):
    """Central hardware management node"""
    
    def __init__(self):
        super().__init__('hardware_manager')
        
        # System state
        self.hardware_state = HardwareState.INITIALIZING
        self.component_status = {
            'arduino': 'UNKNOWN',
            'camera': 'UNKNOWN',
            'lidar': 'UNKNOWN'
        }
        
        # Setup ROS interfaces
        self._setup_publishers()
        self._setup_subscribers()
        
        # Start monitoring
        self._start_monitoring()
        
        self.get_logger().info('Hardware Manager initialized')
    
    def _setup_publishers(self):
        """Setup ROS publishers"""
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Create timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self._publish_system_status)
        self.diagnostics_timer = self.create_timer(2.0, self._publish_diagnostics)
    
    def _setup_subscribers(self):
        """Setup ROS subscribers for component status"""
        self.arduino_status_sub = self.create_subscription(
            String, 'arduino_status', self._arduino_status_callback, 10)
        
        self.camera_status_sub = self.create_subscription(
            String, 'camera_status', self._camera_status_callback, 10)
        
        self.lidar_status_sub = self.create_subscription(
            String, 'lidar_status', self._lidar_status_callback, 10)
        
        # Monitor data flow
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, 'image_raw', self._image_callback, 10)
    
    def _start_monitoring(self):
        """Start system monitoring"""
        self.monitoring_thread = threading.Thread(target=self._monitor_system)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
    
    def _monitor_system(self):
        """Monitor overall system health"""
        while rclpy.ok():
            self._update_system_state()
            time.sleep(0.5)
    
    def _update_system_state(self):
        """Update overall system state based on component status"""
        error_components = [comp for comp, status in self.component_status.items() 
                          if status in ['ERROR', 'DISCONNECTED']]
        
        if error_components:
            if self.hardware_state != HardwareState.ERROR:
                self.hardware_state = HardwareState.ERROR
                self.get_logger().error(f'System error - failed components: {error_components}')
        else:
            ready_components = [comp for comp, status in self.component_status.items() 
                              if status in ['CONNECTED', 'ACTIVE', 'READY']]
            
            if len(ready_components) >= 2:  # At least 2 components working
                if self.hardware_state != HardwareState.READY:
                    self.hardware_state = HardwareState.READY
                    self.get_logger().info('System ready - all critical components online')  
  
    def _arduino_status_callback(self, msg):
        """Handle Arduino status updates"""
        self.component_status['arduino'] = msg.data
        self.get_logger().debug(f'Arduino status: {msg.data}')
    
    def _camera_status_callback(self, msg):
        """Handle camera status updates"""
        self.component_status['camera'] = msg.data
        self.get_logger().debug(f'Camera status: {msg.data}')
    
    def _lidar_status_callback(self, msg):
        """Handle LiDAR status updates"""
        self.component_status['lidar'] = msg.data
        self.get_logger().debug(f'LiDAR status: {msg.data}')
    
    def _odom_callback(self, msg):
        """Monitor odometry data flow"""
        # Could add data validation here
        pass
    
    def _scan_callback(self, msg):
        """Monitor scan data flow"""
        # Could add scan data validation here
        pass
    
    def _image_callback(self, msg):
        """Monitor image data flow"""
        # Could add image data validation here
        pass
    
    def _publish_system_status(self):
        """Publish overall system status"""
        status_msg = String()
        status_msg.data = self.hardware_state.value
        self.system_status_pub.publish(status_msg)
    
    def _publish_diagnostics(self):
        """Publish detailed diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # System-level diagnostics
        system_diag = DiagnosticStatus()
        system_diag.name = 'hardware_manager'
        system_diag.hardware_id = 'dojo_robot'
        
        if self.hardware_state == HardwareState.READY:
            system_diag.level = DiagnosticStatus.OK
            system_diag.message = 'All systems operational'
        elif self.hardware_state == HardwareState.ERROR:
            system_diag.level = DiagnosticStatus.ERROR
            system_diag.message = 'System errors detected'
        else:
            system_diag.level = DiagnosticStatus.WARN
            system_diag.message = f'System state: {self.hardware_state.value}'
        
        # Add component status as key-value pairs
        for component, status in self.component_status.items():
            kv = KeyValue()
            kv.key = f'{component}_status'
            kv.value = status
            system_diag.values.append(kv)
        
        diag_array.status.append(system_diag)
        self.diagnostics_pub.publish(diag_array)
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
        self.hardware_state = HardwareState.ERROR
        
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
    
    def shutdown_hardware(self):
        """Graceful hardware shutdown"""
        self.get_logger().info('Shutting down hardware systems')
        self.hardware_state = HardwareState.SHUTDOWN

def main(args=None):
    rclpy.init(args=args)
    
    try:
        hardware_manager = HardwareManager()
        
        # Use multi-threaded executor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(hardware_manager)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            hardware_manager.shutdown_hardware()
        finally:
            executor.shutdown()
            
    except Exception as e:
        print(f'Hardware Manager error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()