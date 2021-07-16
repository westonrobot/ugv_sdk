# UGV SDK

![GitHub Workflow Status](https://github.com/westonrobot/wrp_sdk/workflows/Cpp/badge.svg)
![GitHub Workflow Status](https://github.com/westonrobot/wrp_sdk/workflows/ROS/badge.svg)

## Introduction

This software package provides a C++ interface to communicate with the mobile platforms from Weston Robot and AgileX Robotics, for sending commands to the robot and receiving the latest robot state. The repository is a joint effort by the development teams at Weston Robot (Singapore) and AgileX Robotics (China).

- Copyright (c) 2020 [Weston Robot](https://www.westonrobot.com/) 
- Copyright (c) 2020 [AgileX Robotics](http://www.agilex.ai/?lang=zh-cn)

Please create an issue on Github at https://github.com/westonrobot/ugv_sdk/issues if you encounter any problems when using the packages.

**Supported robot platforms**

* Scout
* Scout Mini
* Hunter
* Tracer
* Bunker
* Ranger Mini

**Supported environments**

* Architecture: x86_64/arm64
* OS: Ubuntu 16.04/18.04/20.04
* ROS: Kinetic/Melodic/Noetic

It should also work in other similar Linux environments but only the above listed environments are regularly tested.

**Communication protocol**

|       Robot       | Protocol V1 | Protocol V2 | UART  |  CAN  | Support Status |
| :---------------: | :---------: | :---------: | :---: | :---: | :------------: |
|     Scout 1.0     |      Y      |      -      | TODO  |   Y   |  Discontinued  |
|     Scout 2.0     |      Y      |      Y      | TODO  |   Y   |     Active     |
| Scout Mini (Skid) |      Y      |      Y      |   -   |   Y   |     Active     |
| Scout Mini (Omni) |      Y      |      Y      |   -   |   Y   |     Active     |
|    Hunter 1.0     |      Y      |      Y      |   -   |   Y   |     Active     |
|    Hunter 2.0     |      -      |      Y      |   -   |   Y   |     Active     |
|      Bunker       |      Y      |      Y      |   -   |   Y   |     Active     |
|      Tracer       |      -      |      Y      | TODO  |   Y   |     Active     |
|    Ranger Mini    |      -      |      Y      |   -   |   Y   |     Active     |

**Important Note:** Currently we're transitioning the communication protocol from version 1 to version 2. Please check with Weston Robot or AgileX Robotics to confirm which version your robot is using. 

* V1 Protocol: master branch of scout_ros and ugv_sdk
* V2 Protocol: v2.x branch of scout_ros and ugv_sdk

## Build SDK

### Install dependencies

```
$ sudo apt-get update
$ sudo apt-get install build-essential git cmake libasio-dev
```

### Build the package in catkin workspace

```
$ cd <your-catkin-ws>/src
$ git clone -b next https://github.com/westonrobot/ugv_sdk.git
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

## Demo Code

You can find demo code for each robot in "demo" folder. For example, run the demo for Scout robot

```
$ ./bin/demo_scout_robot can0
```

## Reference

* [CAN command reference in Linux](https://rdu.im/docs/canbus)
