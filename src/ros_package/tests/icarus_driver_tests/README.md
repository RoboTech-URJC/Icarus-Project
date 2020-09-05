# Test Packages

First of all, before to try any of these trial packages, you have to run ``roslaunch icarus_driver icarus_driver.launch`` and gazebo simulation whose instructions are available [here](https://github.com/RoboTech-URJC/Icarus-Project/blob/master/src/ros_package/README.md).

## up_down

### Description

**TAKE OFF** - **LAND**  ICARUS DRIVER EXAMPLE.

This is a simple example to test the state machine and their methods in an easy way.

#### Methods used:

* armDisarm()
* takeoff()
* land()

> consult Icarus driver documentation for more info about methods

### Nodes

* **up_down_node:** The drone armed, take off, reach the max high assigned and land. This example is not iterative so the drone take off, land and finish its task.


<p align="center">
  <img width="500" height="260" src="https://github.com/RoboTech-URJC/Icarus-Project/blob/master/docs/up_down_diagram_1.png">
</p>


<p align="center">
  <img width="400" height="360" src="https://github.com/RoboTech-URJC/Icarus-Project/blob/master/docs/up_down_diagram_2.png">
</p>

Click [here](https://www.youtube.com/watch?v=18O6sHyMoOc&feature=youtu.be) to see a demo.

## simple_mover_drone

### Desciption

This is a trial package used for *icarus_driver* testing. Local movement-related methods are tested.

#### Methods used:

* armDisarm()
* takeoff()
* moveLocalTo()
* turnLocalTo()
* land()

### Nodes

* **hfsm_mover_node:** Arm drone, takeoff, it makes a square and later it lands and the drone is disarmed.

The following states machine is used:

<p align="center">
  <img width="607" height="365" src="https://github.com/RoboTech-URJC/Icarus-Project/blob/master/docs/states_machine_simple_mover_drone.png">
</p>

Click [here](https://www.youtube.com/watch?v=8j3A2Or808Q&feature=youtu.be) to see a demo.
