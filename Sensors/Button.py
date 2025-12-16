import os, sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import wiringpi
import time
import threading

class Button(object):
    def __init__(self, wPi:int = 13):
        """
        Button sensor using wiringpi
        :param wPi: GPIO wPi number (wiringpi numbering). default is 13 (GPIO 40, GPIO1_B0)
        """
        super(Button, self).__init__()

        self.wpi = wPi
        self.pressed = False
        # wiringpi setup
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.wpi, wiringpi.INPUT)

        self.thread = threading.Thread(target=self.is_pressed, args=())
        self.thread.start()

    def is_pressed(self) -> bool:
        """
        Check if the button is pressed
        :return: True if pressed, False otherwise
        """
        while True:
            state = wiringpi.digitalRead(self.wpi)
            self.pressed = True if state else False  # assuming active low
            time.sleep(0.2)

if __name__ == "__main__":
    button = Button()
    while True:
        print("Button pressed:", button.pressed)
        time.sleep(1)