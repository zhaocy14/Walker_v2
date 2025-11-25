from gpiozero import DigitalInputDevice
import time
import threading

class Button(object):
    def __init__(self, pin: int = 40, is_show: bool = False):
        """
        to detect whether pressed
        :param pin: GPIO pin number
        """
        self.button = DigitalInputDevice(pin, pull_up=True)
        self.is_pressed = False
        self.is_show = is_show
        self.thread = threading.Thread(target=self.main_loop, args=())
        self.thread.start()

    def main_loop(self):
        """
        main loop to update the button status
        """
        while True:
            value = self.button.value
            if self.is_show:
                print(f"GPIO1_B0 (BCM 40) 数值：{value}\r", end="")
            time.sleep(0.1)
            if value:
                self.is_pressed = False
            else:
                self.is_pressed = True

if __name__ == "__main__":
    button = Button(pin=40)
