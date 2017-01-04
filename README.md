# mma8451
Simple code example for Adafruit MMA8452 3-axis Accelerometer.

This is a very basic python library for mma8451 accelerometer. 
The library was tested on Raspberry pi model 2A and model 3 connected to the Freescale(c) MMA8451 3-axis accelerometer sensor, purchased from Adafruit(c). This is the Adafruit page regarding our device> https://www.adafruit.com/?q=mma8451&

## Dependencies and related readings:

1. https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/configuring-your-pi-for-i2c
A brief guide on How to Configure Your Pi for I2C
  
2. http://www.nxp.com/doc/MMA8451Q
The MMA8451 Data sheet: Technical data

3. http://www.nxp.com/files/sensors/doc/app_note/AN4076.pdf
Data Manipulation and Basic Settings of the MMA8451, 2, 3Q

# Startup
This section explains how to setup the whole things to test your MMA8451 device on a Raspberry Pi

## Basic environment Preparation
* Grab the following material:
** 1 Raspberry Pi (which could be any model)
** 1 Power supply for Raspberry Pi
** 1 SD card or MicroSD card (depending on your RPi model) preferably a class 10
** 1 HDMI monitor
** 1 USB Mouse
** 1 USB Keyboard
** 1 MMA85451 3-axis accelerometer sensor

Also have available...
** A good internet connection (to access the internet)
** A LAN switch (to connect you RPi to the internet)
** Another PC (either Windows, Linux or Mac will be OK)
** Enough patience :)

## Raspberry Pi preparation and setup
* From you PC, download a Raspbian GNU/Linux 8 (jessie) for your RPi. See download page at: https://www.raspberrypi.org/downloads/
* Transfer/Copy the Operating System (Raspbian GNU/Linux 8 - jessie) into the SD/MicroSD Card. (see instruction at: https://www.raspberrypi.org/documentation/installation/installing-images/)
** Instruction on how to install RPi OS from Windows(R) can be found at: https://www.raspberrypi.org/documentation/installation/installing-images/windows.md
** Instruction on how to install RPi OS from Linux can be found at: https://www.raspberrypi.org/documentation/installation/installing-images/linux.md

Once you have transferred/copied the Raspblian into the SD/MicroSD
* Safely dismount the SD/MicroSD Card from your PC
* Stick the SD/MicroSD Card into your RPi
* Connect the HDMI cable of your monitor to rhe HDMI connetor of your RPi
* Connect the USB keyboard to one of the USB connectors available on your RPi
* Connect the USB mouse to ne of the USB connectors available on your RPi
* Power up your RPi by connecting the microUSB connector of the Power Supply to your RPi
* You can watch the startup process of your RPi on the screen.

When the startup procedure is completed you can begin configure you RPi as described here below.

* On our RPi screen, open a terminal window
* Run the Raspberry Pi configuration tool ```raspi-config``` to apply some basic RPi configuration, at least those mentioned below are recommended (for more information, please consult: https://www.raspberrypi.org/documentation/configuration/raspi-config.md)
** Expand Filesystem : to Ensures that all of the SD card storage is available to the OS
** Change User Password : to change password for the default user (pi)
** Advanced Options => A6 I2C : to enable/Disable automatic loading of I2C kernel module
    '''
    # sudo raspi-config    # Run raspi-config
    # sudo reboot          # after raspi-config terminates, reboot your RPi to allow changes to take effect
    '''

* Run ```rpi-update``` to update your RPi firmware and reboot to allow all changes to take effect.
    ```
    # sudo rpi-update
    # sudo reboot
    ```

* Update all packages and eventually newer kernel version (more info at: https://www.raspberrypi.org/documentation/raspbian/updating.md)
    ```
    # sudo apt-get update
    # sudo apt-get upgrade
    ```

* Install the following packages via shell command and reboot you RPi
    ```
    # sudo apt-get install python-smbus
    # sudo apt-get install i2c-tools
    # sudo reboot
    ```

* Update your RPi firmware (more information can be found at:
