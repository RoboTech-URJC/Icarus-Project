# Description
It is about the driver that control all functionalities of Icarus

# Usage and Functionality of Main Methods
## mavutil.mavlink_connection(port)
This method serves to open a serial, UDP, TCP or file mavlink connection, where "port" is the serial device or 
the UDP ot TCP end point. In each case, port value is:
* **Serial Device:** '/dev/ttyX'
* **End Point:** 'udpin:IP:Port' for UDP conection; or 'tcpin:IP, Port' for tcp conection. 
Example:
```
ardupilot = mavutil.mavlink_connection('udpin:localhost:14551')
```
