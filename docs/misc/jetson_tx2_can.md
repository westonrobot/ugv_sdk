# Enable CAN on Jetson TX2

```
$ sudo modprobe can
$ sudo modprobe mttcan
$ sudo ip link set can0 type can bitrate 500000
$ sudo ip link set up can0
```

Reference:

* https://devtalk.nvidia.com/default/topic/1006762/jetson-tx2/how-can-i-use-can-bus-in-tx2-/post/5166583/#5166583
