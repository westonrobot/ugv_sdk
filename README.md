# Weston Robot Platform SDK

![GitHub Workflow Status](https://github.com/westonrobot/wrp_sdk/workflows/Cpp/badge.svg)
![GitHub Workflow Status](https://github.com/westonrobot/wrp_sdk/workflows/ROS/badge.svg)

Copyright (c) 2020 [WestonRobot](https://www.westonrobot.com/)

## Introduction

This software package provides a C++ interface to communicate with the mobile platforms from Weston Robot, for sending commands to the robot and receiving the latest robot state. 

Supported robot platforms

* **Scout**: skid-steer mobile base
* **Hunter**: ackermann mobile base

Supported environments

* **Architecture**: x86_64/arm64
* **OS**: Ubuntu 16.04/18.04/20.04
* **ROS**: Kinetic/Melodic/Noetic

It should also work in other similar Linux environments but only the above listed environments are regularly tested.

Generally, you only need to instantiate an object of the robot base class (such as ScoutBase), then use the object to programmatically control the robot. Internally, the base class manages two background threads, one to process CAN/UART messages of the robot state and accordingly update state variables in the robot state data structure, and the other to maintain a 50Hz loop and send the latest command to the robot base. User can iteratively perform tasks in the main thread and check the robot state or set control commands. 

## Build SDK

### Install dependent libraries

You will need to upgrade CMake to v3.13.0 or later. You can follow the [offical instructions](https://apt.kitware.com/). Here is a brief summary:

```
$ sudo apt-get update
$ sudo apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget
$ wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
```

Ubuntu 20.04 

```
$ sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'
```

Ubuntu 18.04 

```
$ sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
```

Ubuntu 16.04 

```
$ sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main'
```

```
$ sudo apt-get update
$ sudo apt-get install kitware-archive-keyring
$ sudo rm /etc/apt/trusted.gpg.d/kitware.gpg
$ sudo apt-get install cmake
```

### I. Use the package with ROS

```
$ cd <your-catkin-ws>/src
$ git clone https://github.com/westonrobot/wrp_sdk.git
$ cd ..
$ catkin_make
```

### II. Use the package without ROS

If you want to build the TUI monitor tool, additionally install libncurses

```
$ sudo apt install libncurses5-dev
```

Configure and build

```
$ cd wrp_sdk 
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Hardware Interface

### Setup CAN-To-USB adapter 
 
1. Enable gs_usb kernel module
    ```
    $ sudo modprobe gs_usb
    ```
2. Bringup can device
   ```
   $ sudo ip link set can0 up type can bitrate 500000
   ```
3. If no error occured during the previous steps, you should be able to see the can device now by using command
   ```
   $ ifconfig -a
   ```
4. Install and use can-utils to test the hardware
    ```
    $ sudo apt install can-utils
    ```
5. Testing command
    ```
    # receiving data from can0
    $ candump can0
    # send data to can0
    $ cansend can0 001#1122334455667788
    ```

Two scripts inside the "./scripts" folder are provided for easy setup. You can run "./setup_can2usb.bash" for the first-time setup and run "./bringup_can2usb.bash" to bring up the device each time you unplug and re-plug the adapter.

### Setup UART

Generally your UART2USB cable should be automatically recognized as "/dev/ttyUSB0" or something similar and ready for use. If you get the error "... permission denied ..." when trying to open the port, you need to grant access of the port to your user accout:

```
$ sudo usermod -a -G dialout $USER
```

You need to re-login to get the change to take effect.


## Run Demo Apps

The demo code expects one parameter for the CAN bus mode.

```
$ ./app_scout_demo can0
```

Both the port name and baud rate need to be provided when using the RS232 interface.

```
$./app_scout_demo /dev/ttyUSB0 115200
```

If you've installed "libncurses5-dev" and built "app_scout_monitor", you can run it in a similar way:

```
$ ./app_scout_monitor can0
```

or

```
$./app_scout_monitor /dev/ttyUSB0 115200
```

Note: the monitor app is not built by default if you use this SDK with ROS.

## Reference

* [CAN command reference in Linux](https://wiki.rdu.im/_pages/Notes/Embedded-System/Linux/can-bus-in-linux.html)
