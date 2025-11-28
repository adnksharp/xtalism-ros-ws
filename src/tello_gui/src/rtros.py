import sys, rclpy, os

from PySide6.QtCore import QThread, QObject, Signal, Slot 

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, BatteryState, Temperature 
from tello_msg.msg import TelloStatus

PROJECT_TOPIC = '/gui/cmd'

class RosSignals(QObject):
    odom_signal = Signal(dict)
    image_signal = Signal(Image)
    battery_signal = Signal(dict)
    temperature_signal = Signal(dict)
    status_signal = Signal(dict)
    

class RosNode(Node):
    def __init__(self, topic=PROJECT_TOPIC, gui_callback=None): 
        super().__init__('gui_node')
        self.apublisher = self.create_publisher(String, topic, 10)
        self.gui_callback = gui_callback
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            1
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery',
            self.battery_callback,
            10
        )
        
        self.temp_sub = self.create_subscription(
            Temperature,
            'temperature',
            self.temperature_callback,
            10
        )

        self.status_sub = self.create_subscription(
            TelloStatus,
            'status',
            self.status_callback,
            10
        )
        
        self.get_logger().info('initialized.')


    def publish(self, key):
        msg = String()
        msg.data = key
        self.apublisher.publish(msg)
        self.get_logger().info(f'GUI PUB -> {msg.data}')

    def odom_callback(self, msg: Odometry):
        if self.gui_callback:
            data = {
                'px': msg.pose.pose.position.x,
                'py': msg.pose.pose.position.y,
                'pz': msg.pose.pose.position.z,
                'ox': msg.pose.pose.orientation.x,
                'oy': msg.pose.pose.orientation.y,
                'oz': msg.pose.pose.orientation.z,
                'ow': msg.pose.pose.orientation.w,
            }
            self.gui_callback('odom', data)

    def image_callback(self, msg: Image):
        if self.gui_callback:
            self.gui_callback('image', msg)
            
    def battery_callback(self, msg: BatteryState):
        if self.gui_callback:
            data = {
                'percentage': msg.percentage,
                'voltage': msg.voltage
            }
            self.gui_callback('battery', data)
            
    def temperature_callback(self, msg: Temperature):
        if self.gui_callback:
            data = {
                'temperature': msg.temperature 
            }
            self.gui_callback('temperature', data)

    def status_callback(self, msg: TelloStatus):
        if self.gui_callback:
            data = {
                'barometer_cm': msg.barometer, 
                'flight_time': msg.fligth_time,
                'wifi_snr': msg.wifi_snr,
            }
            self.gui_callback('status_extra', data)


class RosThread(QThread):
    def __init__(self, topic=PROJECT_TOPIC):
        super().__init__()
        self.signals = RosSignals() 
        rclpy.init(args=None)
        self.node = RosNode(topic, self.data_callback)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

    def data_callback(self, data_type: str, data):
        if data_type == 'odom':
            self.signals.odom_signal.emit(data)
        elif data_type == 'image':
            self.signals.image_signal.emit(data)
        elif data_type == 'battery':
            self.signals.battery_signal.emit(data)
        elif data_type == 'temperature':
            self.signals.temperature_signal.emit(data)
        elif data_type == 'status_extra':
            self.signals.status_signal.emit(data)

    def run(self):
        self.executor.spin()

    def stop(self):
        self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.quit()
        self.wait()
