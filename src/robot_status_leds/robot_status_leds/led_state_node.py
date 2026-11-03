import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState, Joy
import math
import threading

from .led_controller import MockController, RpiNeoPixelController, hex_to_rgb

class LEDStateNode(Node):
    def __init__(self):
        super().__init__('led_state_node')

        # Parameters (defaults tuned for RPi NeoPixel)
        self.declare_parameter('led_backend', 'rpi_neopixel')   # 'rpi_neopixel' or 'mock'
        self.declare_parameter('led_count', 120)                 # number of LEDs
        self.declare_parameter('led_pin', 18)                  # GPIO number (e.g., 18 -> board.D18)
        self.declare_parameter('brightness', 0.5)              # 0.0 .. 1.0
        self.declare_parameter('static_color', '#00FF00')
        self.declare_parameter('moving_color', '#FF0000')
        self.declare_parameter('blinking_color', '#0000FF')
        self.declare_parameter('startup_color', '#FFA500')
        self.declare_parameter('ready_topic', '/spotarm/named_target')
        self.declare_parameter('velocity_threshold', 0.01)
        self.declare_parameter('moving_timeout_ms', 500)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('joy_timeout_s', 2.0)
        self.declare_parameter('joint_states_timeout_s', 3.0)
        self.declare_parameter('blink_fast_half_period_s', 0.25)
        self.declare_parameter('blink_slow_half_period_s', 0.4)
        self.declare_parameter('pulse_period_s', 2.0)

        # Read parameters
        self.led_backend = self.get_parameter('led_backend').value
        self.led_count = self.get_parameter('led_count').value
        self.led_pin = self.get_parameter('led_pin').value
        self.brightness = self.get_parameter('brightness').value
        self.static_color_hex = self.get_parameter('static_color').value
        self.moving_color_hex = self.get_parameter('moving_color').value
        self.blinking_color_hex = self.get_parameter('blinking_color').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.moving_timeout_ms = self.get_parameter('moving_timeout_ms').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.joy_timeout_s = self.get_parameter('joy_timeout_s').value
        self.joint_states_timeout_s = self.get_parameter('joint_states_timeout_s').value
        self.blink_fast_half_period_s = self.get_parameter('blink_fast_half_period_s').value
        self.blink_slow_half_period_s = self.get_parameter('blink_slow_half_period_s').value
        self.pulse_period_s = self.get_parameter('pulse_period_s').value

        # runtime state
        self.last_moving_ts = 0.0
        self.last_joint_state_ts = 0.0
        self.last_joy_ts = 0.0
        self.is_moving = False
        self.led_state = None
        self.lock = threading.Lock()
        self.last_joint_vels = []

        # init controller
        try:
            if self.led_backend == 'rpi_neopixel':
                self.controller = RpiNeoPixelController(
                    led_count=self.led_count,
                    pin=self.led_pin,
                    brightness=self.brightness,
                    auto_write=False
                )
            else:
                self.controller = MockController(led_count=self.led_count)
        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize LED controller: {str(e)}. Falling back to mock')
            self.controller = MockController(led_count=self.led_count)

        # compute color tuples
        try:
            self.static_color = hex_to_rgb(self.static_color_hex)
        except Exception as e:
            self.get_logger().error(
                f'Invalid static_color parameter {self.static_color_hex}: {e}')
            self.static_color = (0, 0, 255)

        try:
            self.moving_color = hex_to_rgb(self.moving_color_hex)
        except Exception as e:
            self.get_logger().error(
                f'Invalid moving_color parameter {self.moving_color_hex}: {e}')
            self.moving_color = (0, 255, 0)

        try:
            self.blinking_color = hex_to_rgb(self.blinking_color_hex)
        except Exception as e:
            self.get_logger().error(
                f'Invalid blinking_color parameter {self.blinking_color_hex}: {e}')
            self.blinking_color = (0, 0, 255)

        self.ready_topic = self.get_parameter('ready_topic').value

        try:
            self.startup_color = hex_to_rgb(
                self.get_parameter('startup_color').value)
        except Exception as e:
            self.get_logger().error(
                f'Invalid startup_color parameter: {e}')
            self.startup_color = (255, 165, 0)


        # subscribe to joint_states topic
        self.sub = self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 10)

        self.sub = self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 10)

        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_cb, 10)

        # periodic timer to update LEDs
        period = 1.0 / float(self.update_rate_hz)
        self.timer = self.create_timer(period, self.update_leds)

        self.get_logger().info(
            f'led_state_node initialized: backend={self.led_backend} '
            f'pin={self.led_pin} count={self.led_count}')



    def joint_state_cb(self, msg: JointState):
        # joint velocities array may be empty depending on publisher; guard
        self.get_logger().info(f"JOINT CALLBACK RECEIVED: velocity={list(msg.velocity)}")
        self.get_logger().info(f"JOINT CALLBACK RECEIVED: velocity={list(msg.velocity)}")
        with self.lock:
            self.last_joint_state_ts = self.get_clock().now().nanoseconds / 1e9
        if msg.velocity:
            with self.lock:
                self.last_joint_vels = list(msg.velocity)
                max_vel = max(abs(v) for v in self.last_joint_vels)
                
                self.get_logger().info(
                    f"MAX VELOCITY={max_vel}, THRESHOLD={self.velocity_threshold}"
                )   

                now = self.get_clock().now().nanoseconds / 1e9
                if max_vel > self.velocity_threshold:
                    self.last_moving_ts = now

    def joy_cb(self, msg: Joy):
        with self.lock:
            self.last_joy_ts = self.get_clock().now().nanoseconds / 1e9




    def detect_moving(self):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            if (now - self.last_moving_ts) * 1000.0 <= self.moving_timeout_ms:
                return True
            return False



    def compute_state(self, now):
        if self.detect_moving():
            return 'moving'

        with self.lock:
            joint_states_age = now - self.last_joint_state_ts
            joy_age = now - self.last_joy_ts

        if joint_states_age > self.joint_states_timeout_s:
            return 'no_control'

        if self.count_subscribers(self.ready_topic) == 0:
            return 'calibrating'

        if joy_age > self.joy_timeout_s:
            return 'connecting'

        return 'ready'


    def update_leds(self):
        now = self.get_clock().now().nanoseconds / 1e9
        state = self.compute_state(now)

        if state != self.led_state:
            self.led_state = state
            self.get_logger().info(f'LED state changed: {state}')

        if state == 'moving':
            on = int(now / self.blink_fast_half_period_s) % 2 == 0
            color = self.moving_color if on else (0, 0, 0)
        elif state == 'calibrating':
            on = int(now / self.blink_slow_half_period_s) % 2 == 0
            color = self.startup_color if on else (0, 0, 0)
        elif state == 'connecting':
            on = int(now / self.blink_slow_half_period_s) % 2 == 0
            color = self.static_color if on else (0, 0, 0)
        elif state == 'ready':
            color = self.startup_color
        else:
            color = self.static_color

        try:
            self.controller.set_color(color)
        except Exception as e:
            self.get_logger().error(f'Failed to set color: {e}')


    def destroy_node(self):
        try:
            self.controller.close()
        except Exception:
            pass
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = LEDStateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
