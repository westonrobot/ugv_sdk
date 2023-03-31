# UGV SDK

![GitHub Workflow Status](https://github.com/westonrobot/ugv_sdk/workflows/Cpp/badge.svg)
![GitHub Workflow Status](https://github.com/westonrobot/ugv_sdk/workflows/ROS/badge.svg)

## Introduction

This software package provides a C++ interface to communicate with the mobile platforms, for sending commands to the
robot and receiving the latest robot state. The repository is a joint effort by the development teams at Weston Robot (
Singapore) and AgileX Robotics (China).

- Copyright (c) 2020-2023 [Weston Robot](https://www.westonrobot.com/)
- Copyright (c) 2020-2023 [AgileX Robotics](http://www.agilex.ai/?lang=zh-cn)

Please create an issue on Github at https://github.com/westonrobot/ugv_sdk/issues if you encounter any problems when
using the packages.

## Supported Platforms

### Robot bases

* Scout
* Scout Mini
* Hunter
* Tracer
* Bunker
* Ranger Mini

### Software environments

* Architecture: x86_64/arm64
* OS: Ubuntu 16.04/18.04/20.04
* ROS: Kinetic/Melodic/Noetic

It should also work in other similar Linux environments but only the above listed environments are regularly tested.

### Communication protocols

|       Robot       | Protocol V1 | Protocol V2 | UART | CAN | Support Status |
|:-----------------:|:-----------:|:-----------:|:----:|:---:|:--------------:|
|     Scout 1.0     |      Y      |      -      |  N   |  Y  |  Discontinued  |
|     Scout 2.0     |      Y      |      Y      |  N   |  Y  |     Active     |
| Scout Mini (Skid) |      Y      |      Y      |  -   |  Y  |     Active     |
| Scout Mini (Omni) |      Y      |      Y      |  -   |  Y  |     Active     |
|    Hunter 1.0     |      Y      |      Y      |  -   |  Y  |     Active     |
|    Hunter 2.0     |      -      |      Y      |  -   |  Y  |     Active     |
|      Bunker       |      Y      |      Y      |  -   |  Y  |     Active     |
|      Tracer       |      -      |      Y      |  N   |  Y  |     Active     |
|    Ranger Mini    |      -      |      Y      |  -   |  Y  |     Active     |

**Important note:** The main branch of this repository supports all Scout variants and both V1 and V2 protocol. Earlier
versions of the SDK can be found in v1.x and v2.x branch for V1 and V2 protocol support respectively. These two branches
are not
actively maintained any longer and are only kept for reference.

* V1 Protocol: v1.x branch of scout_ros and ugv_sdk
* V2 Protocol: v2.x branch of scout_ros and ugv_sdk

## Build SDK

### Install dependencies

```
$ sudo apt-get update
$ sudo apt-get install build-essential git cmake libasio-dev
```

### Build the package in a catkin workspace

```
$ cd <your-catkin-ws>/src
$ git clone https://github.com/westonrobot/ugv_sdk.git
$ cd ..
$ catkin_make
```

## Setup CAN-To-USB Adapter

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

Two scripts inside the "./scripts" folder are provided for easy setup. You can run "./setup_can2usb.bash" for the
first-time setup and run "./bringup_can2usb.bash" to bring up the device each time you unplug and re-plug the adapter.

## Log CAN Data

In the case that you get an issue with the robot base, you can log the CAN data to a file for further analysis. To do
so, run

```bash
$ candump -l can0
```

If you're using a different CAN interface, replace "can0" with the name of your interface.

The log file can be replayed using the following command

```bash
$ canplayer -I <candump-log-file-name>.log
```

## Sample Code

You can find sample code for each robot in "sample" folder. For example, you can run the demo for Scout robot

```
$ ./bin/demo_scout_robot can0
```

**Important note:**

* **The demo program may command the robot to move!** Please make sure you have the remote controller with you and the
  robot is in a safe place to move around. You can also modify the demo code to disable the motion commands.
* If the robot is not moving as expected, please first check if the remote controller is in manual mode and if the
  E-Stop buttons are released.

## Reference

* [CAN command reference in Linux](https://notes.rdu.im/system/linux/canbus/)
