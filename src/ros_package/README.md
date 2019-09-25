# Install MavROS for this Project

**NOTE:** ROS version ---> Kinetic

**1.** Go to [GitHub Repo](https://github.com/mavlink/mavros), copy his url and make a *git clone* in your ``~/catkin_ws/src`` directory. This should be the command:
```
$ git clone https://github.com/mavlink/mavros
```

**2.** We are going to delete some packages that we don't need and they only create problems because of dependencies. These packages are *mavros_extras* and *test_mavros*. So, type the following commands:
```
$ cd ~/catkin_ws/src/mavros
$ rm -rf test_mavros
$ rm -rf mavros_extras
```

**3.** *mavros* have one dependency with **GeographicLib**, so, to install it, type the following command:
```
$ sudo apt-get install geographiclib-tools
```
Now, go to ''~/catkin_ws/src/mavros/mavros/scripts' and type:
```
$ ./install_geographiclib_datasets.sh
```

**4** The last thing we have to do is install MavLink because MavRos depend on it.

To do this, type:
```
$ sudo apt-get install ros-kinetic-mavlink
```

Now, you can go back to your *catkin_make/* directory and compile all packages typing ``$ catkin_make``
