# Lights Controller
Lights controller has been complete designed for controlling 4 RGB LED raws, in order to provide a better ground-air visualization  with our drone and know at real time what the drone is doing inside its hardware
# Hardware
Hardware consist of a PCB designed in easyeda, 12 transistors **2N2222** , 12 resistors **2k** and **arduino nano** which works as the semi-endebbed microcontroller
# Software
The purpose of the controller mainly is to control with PWM signal the bright of led as well as create a code of colors which allow to controllers to verify what the drone is really doing on air. Programmed in Arduino (C++) the code drives the beheviaour of the lights

 ARDUINO PIN (nano) | COLOR LED |  MOTOR
----------------------|-----------|----
HEAD
3 | R | LEFT & RIGHT
5 | G | LEFT & RIGHT
6 | B | LEFT & RIGHT
TAIL
9 | R | LEFT & RIGHT
10 | G | LEFT & RIGHT
11 | B | LEFT & RIGHT
 
 # Schemes 
<img src="/Electronic_Schemmes/LIGHTS/Scheme_pcb-drone-lights.png" width="500" height="250" >
<img src="/Electronic_Schemmes/LIGHTS/PCB_top_layer.png" width="500" height="250" > 
