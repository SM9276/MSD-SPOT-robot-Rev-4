import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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
        self.declare_parameter('blinking_color', '#0000FF')
        self.declare_parameter('velocity_threshold', 0.01)
        self.declare_parameter('moving_timeout_ms', 500)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('joint_states_topic', '  ')

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

        # runtime state
        self.last_moving_ts = 0.0
        self.is_moving = False
        self.is_blinking = 0
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


        # subscribe to joint_states topic
        self.sub = self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 10)

        self.sub = self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 10)

        # periodic timer to update LEDs
        period = 1.0 / float(self.update_rate_hz)
        self.timer = self.create_timer(period, self.update_leds)

        periodB = 1.0 / float(10) ##float(frequency)
        self.timer = self.create_timer(periodB, self.blink_leds)

        self.get_logger().info(
            f'led_state_node initialized: backend={self.led_backend} '
            f'pin={self.led_pin} count={self.led_count}')



    def joint_state_cb(self, msg: JointState):
        # joint velocities array may be empty depending on publisher; guard
        self.get_logger().info(f"JOINT CALLBACK RECEIVED: velocity={list(msg.velocity)}")
        self.get_logger().info(f"JOINT CALLBACK RECEIVED: velocity={list(msg.velocity)}")
        if msg.velocity:
            with self.lock:
                self.last_joint_vels = list(msg.velocity)
                max_vel = max(abs(v) for v in self.last_joint_vels)
                
                self.get_logger().info(
                    f"MAX VELOCITY={max_vel}, THRESHOLD={self.velocity_threshold}"
                )   

                now = self.get_clock().now().nanoseconds / 1e9
                if max_vel > self.velocity_threshold:
                    if max_vel == 0.3 and self.is_blinking == 0:
                        self.get_logger().info(
                            f"MAX VELOCITY={max_vel}, BLINK VELOCITY=0.3"
                        )  
                        self.is_blinking = 1
                    else:
                        self.last_moving_ts = now
                        self.is_blinking = 0
                    



    def detect_moving(self):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            if (now - self.last_moving_ts) * 1000.0 <= self.moving_timeout_ms:
                return True
            return False



    def update_leds(self):
        if self.is_blinking > 0:
            return
        moving = self.detect_moving()
        if moving != self.is_moving:
            self.is_moving = moving
            state = 'MOVING' if moving else 'STATIC'
            self.get_logger().info(f'Motion state changed: {state}')
        color = self.moving_color if moving else self.static_color
        try:
            self.controller.set_color(color)
        except Exception as e:
            self.get_logger().error(f'Failed to set color: {e}')


    def blink_leds(self):  
        color = self.blinking_color
        try:

            if(self.is_blinking == 1):
                self.controller.set_color((0, 0, 0))                
                self.is_blinking = 2
            elif(self.is_blinking == 2):
                self.controller.set_color(color)
                self.is_blinking = 1


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