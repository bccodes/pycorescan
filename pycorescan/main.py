import os
import time
from datetime import datetime
from enum import Enum

# import pypylon


DESTINATION_DIR = "captures"

class State(Enum):
    ERROR = 1
    BUSY = 2
    READY = 3


def init_GPIO():
    try:
        # import RPi.GPIO as GPIO
        return True
    except RuntimeError:
        print("ERROR: Couldn't access GPIO, No Raspberry Pi detected!")
        return False


def capture_to_dir(directory):
    """
    Creates a file with the current timestamp as its name in the specified directory.

    :param directory: The directory where the file will be created.
    """
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Get the current timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Create a file with the timestamp as the name
    filename = f"{timestamp}.txt"
    filepath = os.path.join(directory, filename)

    with open(filepath, 'w') as f:
        f.write(f"File created at {timestamp}")

    print(f"Created file: {filepath}")


def main_loop():
    state = State.BUSY

    if init_GPIO():
        state = State.READY
    else:
        state = State.ERROR

    while True:
        if (state == State.READY):
            try:
                print("Ready to capture, awaiting input...")
                input()
                capture_to_dir(DESTINATION_DIR)
            except KeyboardInterrupt:
                print("Interrupt received, exiting...")
                break
        print(state)
        time.sleep(1)


if __name__ == "__main__":
    main_loop()
