from __future__ import print_function
from collections import defaultdict
import threading
import time

try:
    import inputs
except ImportError:
    print('Unable to import the `inputs` module. See https://pypi.org/project/inputs/')
    inputs = None


class Gamepad(object):
    """
    Continuously track the state of a gamepad using a thread.
    """

    def __init__(self):
        self.codes = defaultdict(int)

        def update_loop():
            while True:
                self.update()
                time.sleep(0.0001)
        self.thread = threading.Thread(target=update_loop)
        self.thread.setDaemon(True)
        self.thread.start()

    @staticmethod
    def available():
        if inputs is None:
            return False
        if len(inputs.devices.gamepads) == 0:
            print('No gamepads found')
            return False
        return True

    def update(self):
        """
        Process events from the gamepad.
        This blocks until there is an event.
        """
        events = inputs.get_gamepad()
        for event in events:
            if event.ev_type == 'Sync':
                continue
            # Uncomment to print the values of the events for debugging.
            # print(event.ev_type, event.code, event.state)
            self.codes[event.code] = event.state
            # TODO: callbacks for button presses

    def get_command(self):
        """
        Return a tuple of gamepad axis values:
            (velx, vely, velz, yaw_rate, pitch_rate)
        """

        # NOTE: these values come from testing with a Logitech Gamepad F310.
        # Other gamepads may have different layouts and maximums.
        max_hat = 32768.0

        # Vehicle +X (pitch) is out the front, and corresponds to up on the right stick.
        velx = clamp(-1.0 * self.codes['ABS_RY'] / max_hat)

        # Vehicle +Y (roll) is out the left, and corresponds to left on the right stick.
        vely = clamp(-1.0 * self.codes['ABS_RX'] / max_hat)

        # Vehicle +Z (altitude) is out the top, and corresponds to up on the left stick.
        velz = clamp(-1.0 * self.codes['ABS_Y'] / max_hat)

        # Vehicle +yaw is counter-clockwise, and corresponds to left on the left stick.
        yaw_rate = clamp(-1.0 * self.codes['ABS_X'] / max_hat)

        # Gimbal +pitch is looking down, -pitch looks up and zero is straight ahead.
        # Pitch is controlled with left and right triggers on the gamepad.
        # Hold left trigger to look up. Hold right trigger to look down.
        pitch_rate = (self.codes['ABS_RZ'] - self.codes['ABS_Z']) / 255.0

        return (velx, vely, velz, yaw_rate, pitch_rate)


def clamp(val, tol=0.05):
    if abs(val) < tol:
        val = 0.0
    return val


def print_all_codes():
    """ Print details from every event emitted by the gamepad. """
    while True:
        events = inputs.get_gamepad()
        for event in events:
            if event.ev_type == 'Sync':
                continue
            print(event.ev_type, event.code, event.state)


if __name__ == '__main__':
    print_all_codes()
