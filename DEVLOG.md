This repo contains the python code to power the soil core scanner.

This readme will also document some of my progress in development.

Benjamin Worth, 2024

INTRODUCTION
------------
The task at hand is to enable a prototype soil core
scanner to capture images to a USB storage device. The prototype has been
designed and built, but needs all electrics to be installed and connected.

HARDWARE
--------
The provided equipment includes:

- Seeed Odyssey x86 computer
- 2x Basler acA1300-200uc USB3 Cameras
https://www.baslerweb.com/en/shop/aca1300-200uc
- 2x Halo (ring) lights
- 2x UV lights
- Core scanner prototype platform

The prototype platform is made from 3d printed and aluminium components.

Anticipated additional hardware requirements include:

- Push-button to trigger capture
- Multi-color LED to signal device ready or error state
- Power circuit to power the computer, lights and cameras
- Reed switch to detect the physical state of the doors
- Relays to toggle the power to lights with GPIO
- USB hub to connect all devices (camera, camera, relays, led, (button?))

PROPOSED SOLUTION
-----------------
I intend to write a Python script to control the flow of the operation of the 
device, including:

- Detecting the presence of a USB storage (and automatically mounting it)
- Indicating the state of the device via the multi-color LED
- Receiving input from the user via push button to initiate capture
- Detecting the physical state of the device for safety
- Powering on the lights under safe conditions via GPIO
- Capturing via USB3 cameras and storing to USB

One last thing: the device should be as easy to reproduce as possible, in case
someone would like to make another one after I have gone away. Let's see how we
go!

GETTING STARTED: WEEKS 1 & 2
----------------------------
My initial setup for this project has included installing Ubuntu 22.04 onto the 
Seeed (likely how I'll refer to the embeddable computer from here on). The 
slightly older LTS version of Ubuntu has been chosen in hopes that it may make
interacting with the included Raspberry Pi GPIO pins an easier process - Seeed
(the company that makes this computer) includes instructions on their website
for Ubuntu versions up until 22.04.

Seeed documentation:
https://wiki.seeedstudio.com/ODYSSEY-X86J4105-GPIO/

I have opted to give the device a fixed IP to allow it to be patched straight
into my laptop. I have then enabled masquerading on the laptop to enable the
Seeed to connect to the internet. It should only need internet for development
and updating, so hopefully this is fine. the Seeed has the fixed IP 192.168.8.8,
and I have temporarily given my laptop ethernet port the IP 192.168.8.2.

Instructions for the masquerading:
https://securitynetworkinglinux.com/how-to-masquerade-nat-ipv4-traffic-using-ufw-on-ubuntu-cli/

Creating a systemd service to start a python script:
https://www.raspberrypi-spy.co.uk/2015/10/how-to-autorun-a-python-script-on-boot-using-systemd/

I have successfully gotten the USB to automatically mount on the Seeed by adding
a systemd service that calls a shell script. The script calls udisks2 to mount
the USB mass storage under /mnt/usb.

Next steps:
- Testing the usb3 cameras
- test the Seeed GPIO
- write the control script to poll usb and busy/error state if not ready

WEEK 3
------
Updated requirements following meeting with Anton:
- need to view camera preview feed from remote device
Solution:
- use ros2 and serve a WIFI access point from the Seeed
- connect to the access point from the dashboard (remote pc)
- interact with the device through foxglove GUI

Progress since last update:
- Set up ros2 package with usb_cam, image_saver and foxglove_bridge
- tested foxglove-studio for viewing camera feed and sending capture requests

Tasks this week:
- solve power circuit requirements
- put in shopping list for parts
- test basler cameras with usb_cam ros package

Power needs:
------------
- Seeed odyssey: 1x 12v 2a
- Bright UV lights: 2x 24v 1.25a
- Halo lights: 2x 12v
- Basler USB3 cameras: 2x 24v (usb power option)

Due to the new workflow process, the user may interact with the scanner device 
and Seeed computer through a handheld barcode scanner and possibly by 
connecting and disconnecting a USB mass storage device. The remaining 
operations should be performed on the remote dashboard - connected via wifi and
communicating via ROS2 messages and services.

Circuit Ideas:
(240v AC input)
-> step down to 24v
(24v DC)
-> UV light
-> UV light
-> step down to 12v
(12v DC)
-> Halo light
-> Halo light
-> Seed Odyssey
-> Basler camera
-> Basler camera

Shopping list:
--------------
- USB relay board, <=4x relays, 24v max
https://numato.com/product/8-channel-usb-relay-module/ 
- usb3.0 hub, <=4x ports (no power needed)
https://core-electronics.com.au/bonelk-long-life-usb-a-to-4-port-usb-3-0-slim-hub-space-grey.html
https://au.element14.com/connekt-gear/25-0058/usb-3-hub-4-port-bus-powered/dp/3498596
- 2x usb micro b cables for camera data
https://core-electronics.com.au/usb-3-0-micro-b-cable-1m.html
- power supply to 24v, (7.5a?) 
https://au.element14.com/xp-power/ahm180ps24c2-8/adaptor-ac-dc-medical-24v-7-5a/dp/2319716
- step-down to 12v (4.5a?)
https://core-electronics.com.au/12v-4-5a-step-down-voltage-regulator-d36v50f12.html
https://au.element14.com/mascot/5060-24-12v/converter-dc-to-dc-12v-41w/dp/1183940
- 3x reed switches + actuators
https://au.element14.com/littelfuse/57045-000/magnetic-actuator-reed-switch/dp/4209275
https://au.element14.com/littelfuse/59165-1-u-00-a/reed-switch-spst-no-5-1mm-th/dp/4254218
- 3x reed switches

WEEK 4
------
Ordered parts last Thursday! woohoooo

Tasks this week (waiting for parts):
- Set up ROS2 launch scripts
- begin writing capture node
- research solutions for barcode scanner node

ROS2 Launch Patterns:
---------------------
The plan here is to have a single launch file (ros2 python) that can run 
both camera driver nodes and another node to control the capture sequence.
The launch script also needs to bring up a node to control a barcode
scanner.

The capture node needs to be written - it should run two image saver nodes
pointed at the image_raw output of each camera. The images should be saved
under a path/filename indicating the lighting mode (halo or uv) and the time
and date of capture and the most recent barcode scanned.

The Foxglove dashboard should also display the information of most recent
barcode scanned.

Launch plan:
- left camera node
- right camera node
- barcode scanner node
- capture control node
- foxglove-bridge node

Reflecting
----------
A sticking point is passing the currently active barcode to the image saver to 
be used in the filename. currently seeking a solution, working on launching
a capture control node which then starts 2 or 4 image saver nodes with the 
passed in label as a parameter. not functional atm because the nodes 
cannot start under a different rclpy context, i.e. 
rclpy._rclpy_pybind11.RCLError: failed to create client: rcl node's context is invalid, at ./src/rcl/node.c:428

WEEK 5
------
TODO
- Write action client for the basler driver to make 'capture' service available.
- Capture service call should take a string message to trigger capture,
then save an image under that string name in a known path.
String msg from foxglove -> capture service node -> basler action server
image msg from basler action -> capture service node -> write out to file ->
return success

Hope that makes sense later!

Progress:
- Capture node is receiving CaptureRequest messages on /capture and saving
images to disk. Code added to write commands over serial to control relays also.

WEEK 6
------
TODO: 
- Build power circuit.
- Write shopping list for remaining components (case, plugs etc)
- Install OS on seeed odyssey (ubuntu 22.04 desktop)

Power Plan:
24v 7.5a supply ->
- female circular plug
<- 24v power rail
- -> UV LED 1
- -> UV LED 2
- -> Numato Relay Box
- -> 12v 3.5a stepdown
<- 12v power rail
- -> Ring LED 1
- -> Ring LED 2
- -> Basler Cam 1
- -> Basler Cam 2

12v 2a supply ->
- -> Seeed Odyssey

Need to order:
- male mini-din 4 pin (to replace 24v pack plug)
- constant current supply for uv led
- 2x constant current supply for ring led

WEEK 7
------
Done recently:
- Replaced connector on 24v power pack

Things to do today:
- Wire up 24v and 12v circuits
- Order barcode scanner and RCDs (constant current sources)
https://au.element14.com/opticon/c37-usb/barcode-ccd-usb-black/dp/2536869
- Investigate ros2 CameraInfo
- Acquire figure-8 plug for power pack
https://www.jaycar.com.au/2pin-fig-8-mains-plug-to-iec-c7-female-1-8m-figure-8/p/PS4115
- Write parts list in PARTS.md

Tasks to do soon:
- Test LED switching with usb relays
- Complete power circuit with remaining RCDs and safety loop
- Add barcode scanner ROS functionality
- Put it in a box
- WIN

WEEK 8
------
Done recently:
- Tested switching LEDs with usb relay
- Configured wifi AP on Seeed Odyssey with fixed IP 192.168.0.1

To do today:
- Write ROS2 code to receive trigger message from barcode scanner

[ROS2 Design:]

Node:       CaptureNode
Role:       Receives settings updates and trigger messages, then saves images
System:     - Numato USB serial relay controller
            - USB mass storage write
Messages:   (in) CaptureRequest(segment_id: string)
            (in) ConfigUpdate(core_id: string, exposure1: int, exposure2: int) 
            (out) StatusChatter(ready: bool, msg: string)

Node:       TriggerNode
            - USB barcode scanner read
Role:       Sends CaptureRequest messages every time barcode scanner is used
Messages:   (out) CaptureRequest(segment_id: string)

Launch: 2x Pylon Ros2 Basler Camera driver nodes (one per camera)
Action: GrabImages(exposures: int[])

The user dashboard (currently foxglove studio) will provide compressed streams
from both cameras, as well as the ability to change settings and send capture
requests. It will also display the ready state and message.

[BACKEND INSTALLATION PROCESS]
Planned steps:
- install ubuntu 22.04 (24.04 failed install on Seeed Odyssey)
- apt install ros-humble-desktop ros-dev-tools ros2-humble-foxglove-bridge python3-pip apache2
- install pylon ros2 camera driver & deps
- clone pycorescan repo (and sudo pip install evdev and serial)
- build ros workspace & add source to bashrc
- copy udev rule for barcode scanner to /etc/udev/rules.d/
- add hotspot connection with fixed IP via NetworkManager
- setup apache server by replacing /etc/apache2/apache2.conf
- sudo chmod 755 /media/raptor
- disable ufw with systemctl
- add service to systemctl

[FRONTEND INSTALLATION PROCESS]
Planned steps:
- install foxglove-studio
- connect to wifi AP
- open foxglove on 192.168.0.1:8765
- open browser on 192.168.0.1:80

[HARDWARE INSTALLATION PROCESS]
240v(24v(uv1, uv2, relaybox, 12v(ring1, ring2, cam1, cam2)))
240v(12v(backend pc))
pc(usb hub(relaybox, cam1, cam2))

WEEK 9
------
- wrote the barcode scanner module with python and evdev library
- udev rule in linux is also added to allow us to take ownership of the scanner

WEEK 10
-------
- bunch of learning assignments at work

WEEK 11
-------
- brought the model home to continue working on hardware
- tbh not as much progress here as i would have liked

WEEK 12
-------
- working on frontend redesign

WEEK 13
-------
- redesigned front end in html/css/js (had to learn lots to do this)

WEEK 14
-------
- replaced apache2 with python http.server
- specced and acquired enclosure and cable glands

WEEK 15
-------
Changes to backend:
- no longer using apache server
Changes to frontend:
- no longer using foxglove studio
- user will connect to ap then open browser on 192.168.0.1.8000 instead

Todo this week:
- finish hardware (put it in enclosure)
- add better capability and progress notifications to frontend
- add parameters to backend

[Frontend changes]
- add ros2 connected coloured bar at top
- coloured boxes for availability of:
    barcode scanner
    usb storage
    cameras (x2)
    numato box


