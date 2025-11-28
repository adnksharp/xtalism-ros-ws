import sys, rclpy, os

from PySide6.QtCore import QThread, QObject, Signal, Slot 

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, BatteryState, Temperature 
from tello_msg.msg import TelloStatus
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist

PROJECT_TOPIC = '/gui/cmd'

DEFAULT_SPEED = 30.0
LOW_SPEED = 10.0
BOOST_SPEED = 70.0

class RosSignals(QObject):
    odom_signal = Signal(dict)
    image_signal = Signal(Image)
    battery_signal = Signal(dict)
    temperature_signal = Signal(dict)
    status_signal = Signal(dict)
    

class RosNode(Node):
    def __init__(self, topic=PROJECT_TOPIC, gui_callback=None): 
        super().__init__('gui_node')
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

        #self.apublisher = self.create_publisher(String, topic, 10)
        self.cmd_publisher = self.create_publisher(String, topic, 10)
        self.twist_publisher = self.create_publisher(Twist, 'control', 1)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 1)
        self.land_publisher = self.create_publisher(Empty, 'land', 1)
        self.gui_callback = gui_callback
        
        self.get_logger().info('initialized.')

        self.tello_state = 'LAND'

    def publish(self, key):
        SLOW_SPEED = 5.0  # Para CTRL + key
        NORMAL_SPEED = 10.0 
        FAST_SPEED = 15.0 # Para ALT + key
    
        twist_msg = Twist()
        publish_twist = False

        if key.startswith('KEY_'):
            parts = key.split('_')
            mod = parts[1] if len(parts) == 3 else ''
            key_char = parts[-1]
        
            speed = NORMAL_SPEED
            if mod == 'CTRL':
                speed = SLOW_SPEED
            elif mod == 'ALT':
                speed = FAST_SPEED

            if key_char == 'W': twist_msg.linear.y = speed
            elif key_char == 'S': twist_msg.linear.y = -speed
            elif key_char == 'A': twist_msg.linear.x = -speed
            elif key_char == 'D': twist_msg.linear.x = speed
            elif key_char == 'Q': twist_msg.angular.z = speed
            elif key_char == 'E': twist_msg.angular.z = -speed
        
            publish_twist = True

        elif key.startswith('GYRO_'):
            delta = int(key.split('_')[1])
            twist_msg.angular.z = delta * 2.0
            publish_twist = True

        elif key.startswith('SCROLL_'):
            parts = key.split('_')

            mod = parts[1] if len(parts) == 3 else ''
            speed = NORMAL_SPEED
            if mod == 'CTRL':
                speed = SLOW_SPEED

            delta = int(parts[-1])
            twist_msg.linear.z = delta * speed
            publish_twist = True

        elif key == 'MOUSE_2':
            if self.tello_state == 'FLYING':
                self.tello_state = 'LAND'
                self.land_publisher.publish(Empty())
                self.get_logger().info('PUB -> Land')
            else:
                self.tello_state = 'FLYING'
                self.takeoff_publisher.publish(Empty())
                self.get_logger().info('PUB -> Takeoff')
        
        if publish_twist:
            self.twist_publisher.publish(twist_msg)
            self.get_logger().info(f'PUB Twist -> Vx:{twist_msg.linear.x}, Vy:{twist_msg.linear.y}, Oz:{twist_msg.linear.z} Oz:{twist_msg.angular.z}')

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
