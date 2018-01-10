# mma8451 - Accel experimental software
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

# Initial Preparation
This section explains how to setup the whole things to test your MMA8451 device on a Raspberry Pi

## Basic environment Preparation
* Grab the following material:
    1. One Raspberry Pi (which could be any model)
    2. One Power supply for Raspberry Pi
    3. One SD card or MicroSD card (depending on your RPi model) preferably a class 10
    4. One HDMI monitor
    5. One USB Mouse
    6. One USB Keyboard
    7. One MMA85451 3-axis accelerometer sensor

* Also have available...
    8. A good internet connection (to access the internet)
    9. A LAN switch (to connect you RPi to the internet) and a RJ45 UTP/STP LAN straight Cable (patch cord)
    10. Another PC (either Windows, Linux or Mac will be OK)
    11. Enough patience and Spirit of Adventure :)

## Raspberry Pi preparation and setup
* From you PC, download a Raspbian GNU/Linux 8 (jessie) for your RPi. See download page at: https://www.raspberrypi.org/downloads/
* Transfer/Copy the Operating System (Raspbian GNU/Linux 8 - jessie) into the SD/MicroSD Card. (see instruction at: https://www.raspberrypi.org/documentation/installation/installing-images/)
    1. Instruction on how to install RPi OS from Windows(R) can be found at: https://www.raspberrypi.org/documentation/installation/installing-images/windows.md
    2. Instruction on how to install RPi OS from Linux can be found at: https://www.raspberrypi.org/documentation/installation/installing-images/linux.md

Once you have transferred/copied the Raspblian into the SD/MicroSD
* Safely dismount the SD/MicroSD Card from your PC
* Stick the SD/MicroSD Card into your RPi
* Connect the HDMI cable of your monitor to rhe HDMI connetor of your RPi
* Connect the USB keyboard to one of the USB connectors available on your RPi
* Connect the USB mouse to ne of the USB connectors available on your RPi
* Connect your RPi to you LAN switch using the LAN Cable (patch cord)
* Power up your RPi by connecting the microUSB connector of the Power Supply to your RPi
* You can watch the startup process of your RPi on the screen.

When the startup procedure is completed you can begin configure you RPi as described here below.

* On our RPi screen, open a terminal window
* Run the Raspberry Pi configuration tool ```raspi-config``` to apply some basic RPi configuration, at least those mentioned below are recommended (for more information, please consult: https://www.raspberrypi.org/documentation/configuration/raspi-config.md)

    1. Expand Filesystem : to Ensures that all of the SD card storage is available to the OS
    2. Change User Password : to change password for the default user (pi)
    3. Advanced Options => A6 I2C : to enable automatic loading of I2C kernel module
    4. Advanced Options => A4 SSH : to enable remote command line access to your Pi using SSH
    '''
    sudo raspi-config    # Run raspi-config
    sudo reboot          # after raspi-config terminates, reboot your RPi to allow changes to take effect
    '''

* Run ```rpi-update``` to update your RPi firmware and reboot to allow all changes to take effect.
    ```
    sudo rpi-update
    sudo reboot
    ```

* Update all packages and eventually newer kernel version (more info at: https://www.raspberrypi.org/documentation/raspbian/updating.md)
    ```
    sudo apt-get update
    sudo apt-get upgrade
    ```
    Please note that the above command takes some time to complete, since it access the internet to download and install ann the needed software.

* Install the following packages via shell command and reboot you RPi
    ```
    sudo apt-get install build-essential python-dev python-smbus python-pip git
    sudo apt-get install i2c-tools
    sudo reboot
    ```

* Update your RPi firmware (useful information can be found at: https://www.raspberrypi.org/forums/viewtopic.php?t=58963)
    ```
    sudo rpi-update 
    ```
    Please note that the above command takes some time to complete, since it access the internet to download and install ann the needed software.
    
# Software installation
This section will guide you to install the software and run it

## Preliminary set up
In order to allow the MMA8451 software to run as a regular user, for example user ```pi``` (or any other user you might prefer), it is important that the software can access some of the system device file appropriately.

In order to do that, please follow these instructions.
* The file ```/sys/module/i2c_bcm2708/parameters/combined``` must be Read/Write for all users
* The file ```/sys/module/i2c_bcm2708/parameters/combined``` must contain ```Y``` (upper-case 'Y')
The above can be achieved by manually setting the requirements via shell command, as follows:

    ```
    sudo chmod 666 /sys/module/i2c_bcm2708/parameters/combined
    sudo echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined
    modprobe i2c_bcm2708
    ```
    
Or by adding the following chunk of code to the file ```/etc/rc.local```

    ```
    if [ -e /sys/module/i2c_bcm2708/parameters/combined ]; then
        chmod 666 /sys/module/i2c_bcm2708/parameters/combined >> /dev/null
        echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined
    fi
    ```
So that the setting will be applied automatically, every your RPi is rebooted.

## MAA8451 Software installation
The installation of the MMA8451 Experimental software is quite easy:
* Download the software from gitub at https://github.com/massixone/mma8451

    ```
    git clone https://github.com/massixone/mma8451
    cd mma8451
    ```
    
* Run the software

    ```
    ./accel.py
    ```
    
# Finally, Have Fun!
