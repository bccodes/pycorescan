import os
import time
from datetime import datetime

DESTINATION_DIR = "captures"

def create_timestamped_file(directory):
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


def await_user_input(directory):
    """
    Waits for the user to press the Enter key, and then creates a timestamped file.

    :param directory: The directory where the file will be created.
    """
    print("Press Enter to create a timestamped file (Ctrl+C to exit).")
    while True:
        try:
            input()
            create_timestamped_file(directory)
        except KeyboardInterrupt:
            print("Interrupt received, exiting...")
            break


if __name__ == "__main__":
    await_user_input(DESTINATION_DIR)
