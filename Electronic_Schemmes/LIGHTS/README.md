# Lights Controller


[![Hardware](https://img.shields.io/static/v1.svg?label=Hardware&message=Tested&color=0ca017&style=flat-square)](https://github.com/RoboTech-URJC/Icarus-Project/blob/master/CONTRIBUTING.md)&nbsp;
[![Version](https://img.shields.io/static/v1.svg?label=Version&message=v1.0&color=0871a9&style=flat-square)](https://github.com/RoboTech-URJC/Icarus-Project/blob/master/CONTRIBUTING.md)&nbsp;



Lights controller has been complete designed for controlling 4 RGB LED raws by pairs (head and tail of the drone), in order to provide a better ground-air visualization  with our drone and know at real time what the drone is doing inside its hardware
### Hardware
Hardware consist of a PCB, 12 transistors **2N2222** , 12 resistors **2kOhm** and **Arduino Nano** which works as the semi-embedded micro-controller
### Software
The purpose of the controller mainly is to control with PWM signal (not implemented yet) the bright of led as well as create a code of colors which allow to controllers to verify what the drone is really doing on air. Programmed in Arduino (C++) the code drives the behavior of the lights through  the serial port.

:warning: Icarus driver method to comunicate with lights controller: `void IcarusDriver::notifyAck(std::string msg)`

Lights behaviors ~ parameter of notifyAck [`std::string msg`]:

- `a`: activate mode ~ green lights
- `b`: flying mode ~ blue lights
- `c`: warning mode ~ red lights
- `p`: pulse mode ~ flash lights --- to control the interval time of each flash change the variable `int pulse_interval`
- `s`: stop mode ~ turn off all lights


### Connections
 ARDUINO PIN | COLOR LED |
----------------------|-----------|
HEAD
3 | R |
5 | G |
6 | B |
TAIL
9 | R |
10 | G |
11 | B |

<p align="center">
  <img width="500" height="350" src="https://github.com/RoboTech-URJC/Icarus-Project/blob/master/docs/lights_schema_resume.png">
</p>


 ### Designs

 - Schema

<p align="center">
  <img width="500" height="350" src="https://github.com/RoboTech-URJC/Icarus-Project/blob/master/docs/Scheme_pcb-drone-lights.png">
</p>


- PCB layout

<p align="center">
  <img width="500" height="350" src="https://github.com/RoboTech-URJC/Icarus-Project/blob/master/docs/PCB_top_layer.png">
</p>
