#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import PWMOutputDevice


class PwmOutputNode(Node):
    def __init__(self):
        super().__init__('end_effector_pwm')
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('gpio_pin', -1)
        self.declare_parameter('frequency_hz', 100.0)
        self.declare_parameter('command_topic', '/end_effector/duty_cycle')

        self.backend = self.get_parameter('backend').value
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.frequency_hz = self.get_parameter('frequency_hz').value
        self.command_topic = self.get_parameter('command_topic').value
        self.output = None

        if self.backend == 'gpiozero':
            if self.gpio_pin < 0 or self.gpio_pin > 27:
                raise ValueError('gpio_pin must be a BCM GPIO number from 0 to 27')
            if self.gpio_pin == 18:
                raise ValueError('GPIO 18 is reserved for the NeoPixel LED strip')
            if not math.isfinite(self.frequency_hz) or self.frequency_hz <= 0:
                raise ValueError('frequency_hz must be a finite number greater than zero')

            self.get_logger().info(
                f'Initializing GPIO PWM output on pin {self.gpio_pin} with frequency {self.frequency_hz} Hz'
            )

            self.output = PWMOutputDevice(
                pin=self.gpio_pin,
                frequency=self.frequency_hz,
                initial_value=0.0,
            )
        elif self.backend != 'mock':
            raise ValueError("backend must be either 'mock' or 'gpiozero'")

        self.subscription = self.create_subscription(
            Float32,
            self.command_topic,
            self.set_duty_cycle,
            10,
        )
        self.get_logger().info(
            f'PWM output ready: backend={self.backend}, topic={self.command_topic}'
        )

    def set_duty_cycle(self, message):
        duty_cycle = float(message.data)

        self.get_logger().info(f'Received duty cycle command: {duty_cycle:.3f}')
        
        if not math.isfinite(duty_cycle) or not 0.0 <= duty_cycle <= 1.0:
            self.get_logger().warning('Ignoring duty cycle outside the range 0.0 to 1.0')
            return

        if self.output is not None:
            self.output.value = duty_cycle
        self.get_logger().info(f'Duty cycle set to {duty_cycle:.3f}')

    def destroy_node(self):
        if self.output is not None:
            self.output.value = 0.0
            self.output.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PwmOutputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()