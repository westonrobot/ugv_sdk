```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -O - | sudo apt-key add -
$ sudo apt-get update
```

```
$ sudo apt-get install -y python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
$ sudo pip install rosdep rosinstall_generator wstool rosinstall
$ sudo apt-get install -y \
     libconsole-bridge-dev liblz4-dev checkinstall cmake \
     python-empy python-nose libbz2-dev \
     libboost-test-dev libboost-dev  libboost-program-options-dev \
     libboost-regex-dev libboost-signals-dev \
     libtinyxml-dev libboost-filesystem-dev libxml2-dev \
     libgtest-dev libpoco-dev
```

```
$ sudo rosdep init
$ rosdep update
```

```
$ rosinstall_generator ros_comm --rosdistro melodic --deps --tar > melodic-ros_comm.rosinstall
$ wstool init -j1 src melodic-ros_comm.rosinstall
```

```
$ # rosdep install --from-paths src --ignore-src --rosdistro melodic -y
$ sudo ./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/melodic -DCMAKE_BUILD_TYPE=Release
```

```
$ source /opt/ros/melodic/setup.bash
```

Additional runtime dependencies:

```
$ sudo apt install -y python-defusedxml python-netifaces
```

```
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
```

Reference:

* [1] https://machinekoder.com/ros-with-debian-stretch-on-the-beaglebone-black-green-blue/
* [2] http://wiki.ros.org/melodic/Installation/Source