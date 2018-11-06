# Description
It is about the driver that control all functionalities of Icarus

# Usage and Functionality of Main Methods
## mavutil.mavlink_connection(port)
This method serves to open a serial, UDP, TCP or file mavlink connection, where "port" is the serial device or 
the UDP ot TCP end point. In each case, port value is:
* **Serial Device:** '/dev/ttyX'
* **End Point:** 'udpin:IP:Port' for UDP conection; or 'tcpin:IP:Port' for tcp conection. 
Example:
```
ardupilot = mavutil.mavlink_connection('udpin:localhost:14551')
```
This method can also receive more parameter that in case thay are not given, thay have a default value. The header of this method is the next one:
```
def mavlink_connection(device, baud=115200, source_system=255, source_component=0,
                       planner_format=None, write=False, append=False,
                       robust_parsing=True, notimestamps=False, input=True,
                       dialect=None, autoreconnect=False, zero_time_base=False,
                       retries=3, use_native=default_native,
                       force_connected=False, progress_callback=None):
```
