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
        self.declare_parameter('solid_color', '#FF0000')
        self.declare_parameter('blinking_color', '#0000FF')
        self.declare_parameter('fading_color', '#FFFF00')
        self.declare_parameter('startup_color', '#FFA500')

        self.declare_parameter('velocity_threshold', 0.01)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('ready_topic', '/spotarm/named_target')


        # Read parameters
        self.led_backend = self.get_parameter('led_backend').value
        self.led_count = self.get_parameter('led_count').value
        self.led_pin = self.get_parameter('led_pin').value
        self.brightness = self.get_parameter('brightness').value 

        self.static_color_hex = self.get_parameter('static_color').value
        self.solid_color_hex = self.get_parameter('solid_color').value
        self.blinking_color_hex = self.get_parameter('blinking_color').value
        self.fading_color_hex = self.get_parameter('fading_color').value
        self.startup_color_hex = self.get_parameter('startup_color').value

        self.velocity_threshold = self.get_parameter('velocity_threshold').value        
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.ready_topic = self.get_parameter('ready_topic').value

        # runtime state
        self.is_moving = False
        self.last_motion_time = 0
        self.led_state = 0 ## 0=solid, 1=blinking, 2=fading
        self.lock = threading.Lock()
        self.timeout = 0.5 # LED state change timeout in seconds

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
            self.static_color = (0, 255, 0)


        try:
            self.solid_color = hex_to_rgb(self.solid_color_hex)
        except Exception as e:
            self.get_logger().error(
                f'Invalid solid_color parameter {self.solid_color_hex}: {e}')
            self.solid_color = (255, 0, 0) 


        try:
            self.blinking_color = hex_to_rgb(self.blinking_color_hex)
        except Exception as e:
            self.get_logger().error(
                f'Invalid blinking_color parameter {self.blinking_color_hex}: {e}')
            self.blinking_color = (0, 0, 255)


        try:
            self.startup_color = hex_to_rgb(self.startup_color_hex)
        except Exception as e:
            self.get_logger().error(
                f'Invalid startup_color parameter {self.startup_color_hex}: {e}')
            self.startup_color = (255, 165, 0)


        try:
            self.fading_color = hex_to_rgb(self.fading_color_hex)
        except Exception as e:
            self.get_logger().error(
                f'Invalid fading_color parameter {self.fading_color_hex}: {e}')
            self.fading_color = (255, 255, 0)


        # subscribe to joint_states topic
        self.sub = self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 10)

        # periodic timer to update LEDs
        period = 1.0 / float(self.update_rate_hz)
        self.timer = self.create_timer(period, self.update_leds)

        periodB = 1.0 / float(10) ##float(frequency)
        self.timer = self.create_timer(periodB, self.blink_leds)

        periodF = 1.0 / float(0.5) ##float(frequency)
        self.timer = self.create_timer(periodF, self.fade_leds)

        self.get_logger().info(
            f'led_state_node initialized: backend={self.led_backend} '
            f'pin={self.led_pin} count={self.led_count}')



    def joint_state_cb(self, msg: JointState):
        # joint velocities array may be empty depending on publisher; guard
        self.get_logger().info(f"JOINT CALLBACK RECEIVED: velocity={list(msg.velocity)}")
        
        if msg.velocity:
            with self.lock:
                self.last_joint_vels = list(msg.velocity)
                max_vel = max(abs(v) for v in self.last_joint_vels)
                
                self.get_logger().info(
                    f"MAX VELOCITY={max_vel}, THRESHOLD={self.velocity_threshold}"
                )   

                now = self.get_clock().now().nanoseconds / 1e9
                if max_vel > self.velocity_threshold and now - self.last_motion_time > self.timeout:
                    if max_vel > 0.01:
                    ##Redundant condiitonal for possible seperation of blinking and moving commands            
                        self.is_moving = True 
                        self.led_state = 1
                        self.last_motion_time = now
                    else:
                        self.is_moving = False
                        self.led_state = 0
                     


    def update_leds(self):
        ready = self.count_subscribers(self.ready_topic) > 0
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_motion_time >= self.timeout:
            if not ready:
                color = self.startup_color 
                state = 'STARTUP'
            elif self.is_moving == False:
                color = self.static_color
                state = 'STATIC'
            elif self.led_state == 1:
                color = self.blinking_color
                state = 'BLINKING'
            elif self.led_state == 2:
                color = self.fading_color            
                state = 'FADING'
            else:
                color = self.solid_color                
                state = 'SOLiD'
            self.get_logger().info(f'Motion state changed: {state}')

            try:
                self.controller.set_color(color)
            except Exception as e:
                self.get_logger().error(f'Failed to set color: {e}')


    def blink_leds(self):  
        try:
            if(self.led_state == 1):
                color = self.controller.get_color()

                if(color == self.blinking_color):
                    self.controller.set_color((0, 0, 0))                
                else:
                    self.controller.set_color(self.blinking_color)

        except Exception as e:
            self.get_logger().error(f'Failed to set color: {e}')


    def fade_leds(self):  
        try:
            color = self.controller.get_color()

            if(color == self.blinking_color):
                self.controller.set_color((0, 0, 0))                
            else:
                self.controller.set_color(self.blinking_color)

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
