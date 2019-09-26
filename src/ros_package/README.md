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

# Install Px4 Simulator

**1.** Download [this file](https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_common_deps.sh).

**2.** Run the script typing ``$ source ubuntu_sim_common_deps.sh`` in the folder where file is located. When script has finished, you will be in s *src* folder inside your home.

**3.** Type ``$ cd Firmware`` to go to this folder. later, run:

```
$ make px4_sitl jmavsim
```

This command will launch the simulator

**!!We have just the simulator installed!!**

![Image](https://github.com/RoboTech-URJC/Icarus-Project/tree/master/docs/simulator_drone.jpg)

but...

To be able to communicate the messages we send to ros topics or services, for example, since our c++ programs, it is necessary to run the netxt launch:

```
$ roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

This is the launch file located inside the *mavros* package previously installed, but, we run it with a new parameter to indicates the access point where simulator is listenning.

At this moment, you can send commands to the simulated drone using ros topics or ros services.

**NOTE:** You can already use simulator command line (terminal). You can find an extended explanation [here](https://dev.px4.io/v1.9.0/en/setup/building_px4.html). In addition, if you type ``help``, a guide of all available commands will be shown.
