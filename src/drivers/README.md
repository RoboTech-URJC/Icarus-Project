# Description
It is about the driver that control all functionalities of Icarus

# Usage and Functionality of Main Methods
Here is explained the main methods used in the driver 'driver-icarus.py' included in pymavlink library.
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
In this case, we use that method to receive the initial parameters of configuration of the drone. This type of message is called "HEARTBEATE", so, if we want to receive this parameter, the sentence to do this will be                                              
``msg = ardupilot.recv_match(type='HEARTBEAT', blocking=True)``. The parameter "blocking" serves to make that the call to this method blocking. 

## recv_msg()
Receive any message from the drone. Example:
```
msg = self.ardupilot.recv_msg()
print msg
```
We can put this code into a loop to read every time the messages the drone sends.

## mav.heartbeat_send()
This method is very important because it serves to coordinate the pc with the drone. That function send a hearbeat message to the drone of the samme way that drone sended it to us and we could to read it using the method "recv_match()" previously mentioned. It is very important that the parameters we send to the drone with this method are the same. The header of this methos is as follows:
```
self.ardupilot.mav.heartbeat_send(type, autopilot, base_mode, custom_mode, system_status, mavlink_version)
```
You can see which are these parameters using the previous function as follows:
```
msg = ardupilot.recv_match(type='HEARTBEAT', blocking=True)
print(msg)
```
## set_mode_apm(mode)
This method serves to set the flight mode of the drone to any of different modes that exist. That modes are:

| Mode | Summary |
| ---- | ------- |
| Acro |  Holds attitude, no self-level   |
| Alt Hold |  Holds altitude and self-levels the roll & pitch   |
| Auto |  Executes pre-defined mission   |
| AutoTune |  Executes pre-defined mission   |
| Brake |  Brings copter to an immediate stop   |
| Circle |  Automatically circles a point in front of the vehicle   |
| Drift |  Like stabilize, but coordinates yaw with roll like a plane   |
| Flip |  Rises and completes an automated flip   |
| FlowHold |  Position control using Optical Flow   |
| Follow |  Follows another vehicle   |
| Guided |  Navigates to single points commanded by GCS   |
| Land |  Reduces altitude to ground level, attempts to go straight down   |
| Loiter |  Holds altitude and position, uses GPS for movements   |
| PosHold |  Like loiter, but manual roll and pitch when sticks not centered   |
| RTL |  Retruns above takeoff location, may aslo include landing   |
| SmartRTL |  RTL, but traces path to get home   |
| Stabilize |  Self-levels the roll and pitch axis   |
| Sport |  Alt-hold, but holds pitch & roll when sticks centered   |
| Throw |  Holds position after a throwing takeoff   |
