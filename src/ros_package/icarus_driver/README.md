# ICARUS DRIVER

#### API of Icarus driver


| NAME  | DESCRIPTION | FUNCTION |
| -------------| ------------- | ------------- |
| set mode  |   |`IcarusDriver::SetMode(std::string mode)`|
| arm / disarm  |  | `IcarusDriver::armDisarm(int arm)`|
| take off  |   |`IcarusDriver::takeoff(double alt)`|
| move to local point  |  |`IcarusDriver::moveLocalTo(double x, double y, double z)`|
| notify an external serial port device  |   |`IcarusDriver::notifyAck(std::string msg)`|


> TODO: landing(), emergency_landing(), move_global_to()

# BOCANEGRA

Bocanegra is a state machine conceived specially for this project and adressed to give the best performace for our purpose.

<!-- <img src="/home/pabloc/Desktop/Github/Icarus-Project/docs/diagram.png" width="600" height="400" > -->

<p align="center">
  <img width="460" height="300" src="/home/pabloc/Desktop/Github/Icarus-Project/docs/diagram.png">
</p>
