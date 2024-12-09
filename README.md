# PyCoreScan

## Overview

PyCoreScan is a soil core scanner designed to capture high-resolution images of soil samples. This document provides instructions for users to connect to the device's frontend and for developers to set up similar devices.

This repo has been developed by Benjamin Worth for an undergraduate internship project with CSIRO Black Mountain and University of Canberra

---

## User Instructions

### Accessing the Frontend

1. **Power the devices on**
   - Plug in the scanner with the 24 volt power pack
   - Plug in the Seeed Odyssey with its provided power cable
   - Make sure the scanner is connected to the (blue) USB3 port on the Seeed Odyssey.
   - Connect a USB storage device to one of the other USB ports on the Seed Odyssey. (Note: The scanner will create a folder on this storage device called "corescans" where images are saved.)

2. **Connect to the Device's Wi-Fi:**
   - On your computer or mobile device, go to Wi-Fi settings.
   - Look for a network named `Raptor-001`.
   - Connect to this network with the password `Raptor-001`

3. **Open the Web Dashboard:**
   - Once connected to the Wi-Fi network, open a web browser.
   - Navigate to `http://192.168.0.1:8000` to access the device's frontend interface.

4. **Troubleshooting**
   - To save images, a USB storage device must be connected.
   - The web dashboard will notify the user if the Seeed Odyssey is not available, or if the USB storage, light controller, or barcode scanner are not available.
   - If the cameras have become disconnected during operation the web dashboard may freeze.
   - To fix most issues, check the physical connections and refresh the Web Dashboard.
   - If the Dashboard shows `ROS2 Disconnected`, restart the Seeed Odyssey. Wait 2 minutes, then refresh the Dashboard.

---

## Developer Setup Instructions

### Hardware Overview

- **Computing Unit:** Seeed Odyssey x86 computer
- **Cameras:** 2x Basler acA1300-200uc USB3 Cameras
- **Lighting:**
  - 1x Numato 8 Relay 24v USB Controller
  - 2x Halo (ring) LEDs with 500mA RCDs
  - 2x UV LEDs with 700mA RCDs
- **Power:**
  - 24v 7.5A AC-DC Power Pack
- For full specs see DEVLOG.md

### Software Setup

1. **Operating System:**
   - Install Ubuntu 22.04 on the Seeed Odyssey

2. **Setup ROS2 & Dependencies:**
   - Update package lists:

     ```bash
     sudo apt-get update
     ```

   - Install required packages:

     ```bash
     sudo apt-get install python3 python3-pip git 
     ```

   - Install ROS2 Humble
     - Follow the instructions [here](https://docs.ros.org/en/humble/Installation.html).
   - Install Basler Pylon SDK:
     - Download the SDK from the [Basler website](https://www.baslerweb.com/en/sales-support/downloads/software-downloads/).
     - Follow the installation instructions provided.
   - Install the Basler ROS2 Driver:
     - Follow the instructions [here](https://github.com/basler/pylon-ros-camera).

3. **Clone the PyCoreScan Repository:**
   - Navigate to the home directory and create the folder `ros2_ws/src`.
   - Clone the PyCoreScan repository:

     ```bash
     git clone https://github.com/bccodes/pycorescan.git
     ```

   - Navigate back to the `ros2_ws` directory
   - Build the ROS2 workspace with `colcon build`.

4. **Install Python Dependencies:**
   - Use pip to install required Python packages:

     ```bash
     sudo pip3 install evdev serial
     ```

5. **Configure Udev Rules:**
   - Copy the provided udev rules to the appropriate directory:

     ```bash
     sudo cp udev/99-pycorescan.rules /etc/udev/rules.d/
     ```

   - Reload udev rules:

     ```bash
     sudo udevadm control --reload-rules
     sudo udevadm trigger
     ```

6. **Set Up Systemd Services:**
   - Copy service files to the systemd directory:

     ```bash
     sudo cp pcs_service/pycorescan.service /etc/systemd/system/
     ```

   - Enable and start the service:

     ```bash
     sudo systemctl enable pycorescan.service
     sudo systemctl start pycorescan.service
     ```

7. **Configure Wi-Fi Access Point:**
   - Use NetworkManager in Ubuntu via nmcli or nmtui (text-based graphical interface) to configure the Hotspot:

   ```bash
   nmcli connection add type wifi ifname wlo2 con-name Hotspot autoconnect yes ssid raptor-001

    nmcli connection modify Hotspot 802-11-wireless.mode ap 802-11-wireless.band bg
    nmcli connection modify Hotspot 802-11-wireless-security.key-mgmt wpa-psk
    nmcli connection modify Hotspot 802-11-wireless-security.proto rsn
    nmcli connection modify Hotspot 802-11-wireless-security.group ccmp
    nmcli connection modify Hotspot 802-11-wireless-security.pairwise ccmp
    nmcli connection modify Hotspot 802-11-wireless-security.psk raptor-001
    nmcli connection modify Hotspot ipv4.method shared ipv4.addresses 192.168.0.1/24
    nmcli connection modify Hotspot ipv6.method ignore
    nmcli connection up Hotspot

    ```

8. **Testing:**
   - Reboot the system to ensure all services start correctly.
   - Connect to the Wi-Fi network and access the dashboard and see the camera previews.

*Note: The previous README content has been moved to `DEVLOG.md` for reference.*

For further information, [Email Ben](u3243222@uni.canberra.edu.au)
