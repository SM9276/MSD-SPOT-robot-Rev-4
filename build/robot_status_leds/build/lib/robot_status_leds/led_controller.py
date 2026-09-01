"""
LED controller backends for NeoPixel on Raspberry Pi (Adafruit CircuitPython NeoPixel)
and a MockController for development/testing.
"""

import logging
logger = logging.getLogger('robot_status_leds.led_controller')

def hex_to_rgb(hex_str):
    hex_str = hex_str.lstrip('#')
    if len(hex_str) == 6:
        r = int(hex_str[0:2], 16)
        g = int(hex_str[2:4], 16)
        b = int(hex_str[4:6], 16)
        return (r, g, b)
    raise ValueError("hex color must be 6 chars, e.g. '#RRGGBB'")

class LEDControllerBase:
    def set_color(self, rgb):
        """Set entire strip to RGB tuple (r,g,b). Implement in subclass."""
        raise NotImplementedError()

    def show(self):
        """Force a show if backend needs it."""
        pass

    def close(self):
        """Clean up resources if needed."""
        pass

class MockController(LEDControllerBase):
    def __init__(self, led_count=30, **kwargs):
        self.led_count = led_count
        logger.info("MockController init: led_count=%d", led_count)

    def set_color(self, rgb):
        logger.info("MockController set_color: rgb=%s, count=%d", rgb, self.led_count)

class RpiNeoPixelController(LEDControllerBase):
    """
    Uses Adafruit CircuitPython NeoPixel library.
    Requires:
        sudo pip3 install rpi_ws281x adafruit-blinka adafruit-circuitpython-neopixel
    On Raspberry Pi the data pin should be GPIO18 (board.D18) by default for correct timing.
    """

    def __init__(self, led_count=30, pin=18, brightness=0.5, auto_write=False, **kwargs):
        try:
            import board
            import neopixel
        except Exception as e:
            logger.exception("Failed to import NeoPixel libraries: %s", e)
            raise

        # Resolve board pin attribute like board.D18
        try:
            board_pin = getattr(board, f"D{pin}")
        except Exception:
            # fallback options
            try:
                board_pin = getattr(board, f"GPIO{pin}")
            except Exception:
                board_pin = getattr(board, "D18")

        self.led_count = int(led_count)
        try:
            b = float(brightness)
            if b < 0.0:
                b = 0.0
            if b > 1.0:
                b = 1.0
            self.brightness = b
        except Exception:
            self.brightness = 0.5

        self.pixels = neopixel.NeoPixel(board_pin, self.led_count, brightness=self.brightness, auto_write=auto_write)
        self.auto_write = auto_write
        logger.info("RpiNeoPixelController init: led_count=%d pin=%s brightness=%.2f auto_write=%s",
                    self.led_count, board_pin, self.brightness, self.auto_write)

    def set_color(self, rgb):
        r, g, b = [int(max(0, min(255, int(x)))) for x in rgb]
        color = (r, g, b)
        self.pixels.fill(color)
        if not self.auto_write:
            self.pixels.show()

    def show(self):
        if not self.auto_write:
            self.pixels.show()

    def close(self):
        try:
            self.pixels.fill((0, 0, 0))
            if not self.auto_write:
                self.pixels.show()
        except Exception:
            pass