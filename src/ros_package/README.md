# Install MavROS for this Project

**NOTE:** ROS version ---> Melodic

**1.** Go to [GitHub Repo](https://github.com/mavlink/mavros), copy its url and make a *git clone* in your ``~/catkin_ws/src`` directory. This should be the command:
```
$ git clone https://github.com/mavlink/mavros
```

Now, you can go back to your *catkin_make/* directory and compile all packages typing ``$ catkin_make``

# Run Quadcopter on Gazebo

**1.** Clone [this repo](https://github.com/PX4/Firmware) outside of catkin_ws.

**2.** Type ``$ cd Firmware`` to go to cloned repo. Later, run:

```
$ DONT_RUN=1 make px4_sitl_default gazebo
```
**3.** Once everything is compiled, go to *Firmware* folder and run:

```
Tools/gazebo_multi_vehicle.sh -n 1
```
if the command above didn't work just try: ``` make px4_sitl_default gazebo```

**This command will launch the simulator!**

but...

To be able to communicate the messages we send to ros topics or services, for example, since our c++ programs, it is necessary to run the netxt launch:

```
$ roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

This is the launch file located inside the *mavros* package previously installed, but, we run it with a new parameter to indicates the access point where simulator is listenning.

At this moment, you can send commands to the simulated drone using ros topics or services.
