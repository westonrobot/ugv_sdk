# UGV SDK

![GitHub Workflow Status](https://github.com/westonrobot/wrp_sdk/workflows/Cpp/badge.svg)
![GitHub Workflow Status](https://github.com/westonrobot/wrp_sdk/workflows/ROS/badge.svg)

## Introduction

This software package provides a C++ interface to communicate with the mobile platforms from Weston Robot and AgileX Robotics, for sending commands to the robot and receiving the latest robot state. The repository is a joint effort by the development teams at Weston Robot (Singapore) and AgileX Robotics (China).

- Copyright (c) 2020 [Weston Robot](https://www.westonrobot.com/) 
- Copyright (c) 2020 [AgileX Robotics](http://www.agilex.ai/?lang=zh-cn)

Please create an issue on Github at https://github.com/westonrobot/ugv_sdk/issues if you encounter any problems in using the packages.

**Important Note:** Currently we're transitioning the communication protocol from version 1 to version 2. Please check with Weston Robot or AgileX Robotics to confirm which version your robot is using. 

* V1 Protocol: master branch of scout_ros and ugv_sdk
* V2 Protocol: v2.x branch of scout_ros and ugv_sdk

**Supported robot platforms**

* **Scout**: skid-steer mobile base
* **Hunter**: ackermann mobile base

**Supported environments**

* **Architecture**: x86_64/arm64
* **OS**: Ubuntu 16.04/18.04/20.04
* **ROS**: Kinetic/Melodic/Noetic

It should also work in other similar Linux environments but only the above listed environments are regularly tested.

**Communication protocol**

| Robot  | Protocol Version |
| :----: | :--------------: |
| Scout  |      V1, V2      |
| Hunter |      V1, V2      |
| Tracer |        V2        |
| Bunker |        V1        |

Generally, you only need to instantiate an object of the robot base class (such as ScoutBase), then use the object to programmatically control the robot. Internally, the base class manages two background threads, one to process CAN/UART messages of the robot state and accordingly update state variables in the robot state data structure, and the other to maintain a 50Hz loop and send the latest command to the robot base. User can iteratively perform tasks in the main thread and check the robot state or set control commands. Advanced users may also use this setup as a reference and maintain the control and monitoring loops in a different way.

## Build SDK

### Install dependent libraries

```
$ sudo apt-get update
$ sudo apt-get install build-essential git cmake
```

### Build the package in catkin workspace

```
$ cd <your-catkin-ws>/src
$ git clone https://github.com/westonrobot/async_port.git
$ git clone -b v2.x https://github.com/westonrobot/ugv_sdk.git
$ cd ..
$ catkin_make
```

## Setup CAN-To-USB adapter 
 
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

## Run Demo Apps

The demo code expects one parameter for the CAN bus mode.

```
$ ./app_scout_demo can0
```

## Reference

* [CAN command reference in Linux](https://wiki.rdu.im/_pages/Notes/Embedded-System/Linux/can-bus-in-linux.html)
