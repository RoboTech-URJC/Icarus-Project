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

## recv_match()
Receive the next MAVLink message. It can receive parameters or not. His header is the next one:
```
def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
```
In this case, we use that method to receive the initial parameters of configuration of the drone. This type of message is called "HEARTBEATE", so, if we want to receive this parameter, the sentence to do this will be ``msg = ardupilot.recv_match(type='HEARTBEAT', blocking=True)``. The parameter "blocking" serves to make that the call to this method blocking. 
