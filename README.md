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
- 2x USB3 Cameras
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
go.


