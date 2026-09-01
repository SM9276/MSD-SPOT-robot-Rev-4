import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        self.declare_parameter('velocity_threshold', 0.01)
        self.declare_parameter('moving_timeout_ms', 500)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('joint_states_topic', 'joint_states')

        # Read parameters
        self.led_backend = self.get_parameter('led_backend').value
        self.led_count = self.get_parameter('led_count').value
        self.led_pin = self.get_parameter('led_pin').value
        self.brightness = self.get_parameter('brightness').value
        self.static_color_hex = self.get_parameter('static_color').value
        self.moving_color_hex = self.get_parameter('moving_color').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.moving_timeout_ms = self.get_parameter('moving_timeout_ms').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value

        # runtime state
        self.last_moving_ts = 0.0
        self.is_moving = False
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
            self.get_logger().error('Failed to initialize LED controller: %s. Falling back to mock', str(e))
            self.controller = MockController(led_count=self.led_count)

        # compute color tuples
        try:
            self.static_color = hex_to_rgb(self.static_color_hex)
        except Exception as e:
            self.get_logger().error('Invalid static_color parameter %s: %s', self.static_color_hex, e)
            self.static_color = (0, 0, 255)

        try:
            self.moving_color = hex_to_rgb(self.moving_color_hex)
        except Exception as e:
            self.get_logger().error('Invalid moving_color parameter %s: %s', self.moving_color_hex, e)
            self.moving_color = (0, 255, 0)

        # subscribe to joint_states topic
        self.sub = self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 10)

        # periodic timer to update LEDs
        period = 1.0 / float(self.update_rate_hz)
        self.timer = self.create_timer(period, self.update_leds)

        self.get_logger().info('led_state_node initialized: backend=%s pin=%s count=%d',
                               self.led_backend, str(self.led_pin), self.led_count)

    def joint_state_cb(self, msg: JointState):
        # joint velocities array may be empty depending on publisher; guard
        if msg.velocity:
            with self.lock:
                self.last_joint_vels = list(msg.velocity)
                max_vel = max(abs(v) for v in self.last_joint_vels)
                now = self.get_clock().now().nanoseconds / 1e9
                if max_vel > self.velocity_threshold:
                    self.last_moving_ts = now

    def detect_moving(self):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            if (now - self.last_moving_ts) * 1000.0 <= self.moving_timeout_ms:
                return True
            return False

    def update_leds(self):
        moving = self.detect_moving()
        if moving != self.is_moving:
            self.is_moving = moving
            self.get_logger().info('Motion state changed: %s', 'MOVING' if moving else 'STATIC')
        color = self.moving_color if moving else self.static_color
        try:
            self.controller.set_color(color)
        except Exception as e:
            self.get_logger().error('Failed to set color: %s', e)

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()