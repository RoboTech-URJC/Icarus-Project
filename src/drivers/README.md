# Description
It is about the driver that control all functionalities of Icarus

# Usage and Functionality of Main Methods
Here is explained the main methods used in the driver 'driver-icarus.py' included in pymavlink library.
## mavutil.mavlink_connection(port)
This method serves to open a serial, UDP, TCP or file mavlink connection, where "port" is the serial device or 
the UDP ot TCP end point. In each case, port value is:
* **Serial Device:** '/dev/ttyX'
* **End Point:** 'udpin:IP:Port' for UDP connection; or 'tcpin:IP:Port' for tcp connection. 
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

For example, to change to "Guided" mode: ``ardupilot.set_mode_apm('GUIDED')`` .

## mav.command_long_send()
Serves to send commands to the drone. The parameters that it receives depend of the command type. [Here](http://ardupilot.org/planner/docs/common-mavlink-mission-command-messages-mav_cmd.html#common-mavlink-mission-command-messages-mav-cmd) are every commands and his usability

# How to Receive Parameters?
To receive parameters we use ``messages[[Param type]].[match]``. For example, if we want to receive the relative altitude of the drone, we will use ``ardupilot.messages['GLOBAL_POSITION_INT'].relative_alt``

How can we know which is the param type we have tu use? We can see all the parameters that the drone sends with ``recv_msg`` inside a loop. that is an example of we receive:
```
RAW_IMU {time_usec : 2420564594, xacc : 0, yacc : 0, zacc : -1001, xgyro : 1, ygyro : 1, zgyro : 1, xmag : 194, ymag : 118, zmag : -538}
SCALED_IMU2 {time_boot_ms : 32485335, xacc : 0, yacc : 0, zacc : -1001, xgyro : 1, ygyro : 1, zgyro : 1, xmag : 194, ymag : 118, zmag : -538}
SCALED_PRESSURE {time_boot_ms : 32485335, press_abs : 945.027038574, press_diff : 0.0, temperature : 3500}
SYS_STATUS {onboard_control_sensors_present : 56753199, onboard_control_sensors_enabled : 39975983, onboard_control_sensors_health : 56753199, load : 0, voltage_battery : 12587, current_battery : 0, battery_remaining : 83, drop_rate_comm : 0, errors_comm : 0, errors_count1 : 0, errors_count2 : 0, errors_count3 : 0, errors_count4 : 0}
POWER_STATUS {Vcc : 5000, Vservo : 0, flags : 0}
MEMINFO {brkval : 0, freemem : 65535, freemem32 : 131072}
MISSION_CURRENT {seq : 0}
GPS_RAW_INT {time_usec : 32485215000, fix_type : 6, lat : -353632564, lon : 1491652320, alt : 584100, eph : 121, epv : 200, vel : 0, cog : 11789, satellites_visible : 10, alt_ellipsoid : 0, h_acc : 200, v_acc : 200, vel_acc : 40, hdg_acc : 0}
NAV_CONTROLLER_OUTPUT {nav_roll : -0.119866870344, nav_pitch : -0.117011599243, nav_bearing : -22, target_bearing : 0, wp_dist : 0, alt_error : -0.657050848007, aspd_error : 0.0, xtrack_error : 0.0}
GLOBAL_POSITION_INT {time_boot_ms : 32485335, lat : -353632563, lon : 1491652315, alt : 584010, relative_alt : 6, vx : 1, vy : -4, vz : 0, hdg : 33785}
LOCAL_POSITION_NED {time_boot_ms : 32485335, x : -0.0935497283936, y : 0.259233921766, z : -0.00685916794464, vx : 0.0173883810639, vy : -0.0487783513963, vz : 0.00074397190474}
SERVO_OUTPUT_RAW {time_usec : 2420564594, port : 0, servo1_raw : 1000, servo2_raw : 1000, servo3_raw : 1000, servo4_raw : 1000, servo5_raw : 0, servo6_raw : 0, servo7_raw : 0, servo8_raw : 0, servo9_raw : 0, servo10_raw : 0, servo11_raw : 0, servo12_raw : 0, servo13_raw : 0, servo14_raw : 0, servo15_raw : 0, servo16_raw : 0}
RC_CHANNELS {time_boot_ms : 32485335, chancount : 16, chan1_raw : 1500, chan2_raw : 1500, chan3_raw : 1000, chan4_raw : 1500, chan5_raw : 1800, chan6_raw : 1000, chan7_raw : 1000, chan8_raw : 1800, chan9_raw : 0, chan10_raw : 0, chan11_raw : 0, chan12_raw : 0, chan13_raw : 0, chan14_raw : 0, chan15_raw : 0, chan16_raw : 0, chan17_raw : 0, chan18_raw : 0, rssi : 0}
ATTITUDE {time_boot_ms : 32485335, roll : -0.00209207204171, pitch : -0.00204223790206, yaw : -0.386602848768, rollspeed : -0.000276305130683, pitchspeed : -0.000244989059865, yawspeed : -0.000729398336262}
SIMSTATE {roll : 0.0, pitch : 0.0, yaw : -0.341762781143, xacc : -2.19979159888e-10, yacc : 2.11345663192e-10, zacc : -9.80665016174, xgyro : -8.40779078595e-43, ygyro : -7.28675201449e-43, zgyro : 1.43051111081e-05, lat : -353632564, lng : 1491652320}
AHRS2 {roll : 0.00179860368371, pitch : 0.00161452894099, yaw : -0.345547944307, altitude : 0.0, lat : 0, lng : 0}
AHRS3 {roll : -0.00209207204171, pitch : -0.00204223790206, yaw : -0.386602848768, altitude : 584.010009766, lat : -353632563, lng : 1491652315, v1 : 0.0, v2 : 0.0, v3 : 0.0, v4 : 0.0}
VFR_HUD {airspeed : 0.0, groundspeed : 0.0517849698663, heading : 337, throttle : 0, alt : 584.010009766, climb : -0.00074397190474}
AHRS {omegaIx : -0.00153755501378, omegaIy : -0.00152305862866, omegaIz : -0.00194238708355, accel_weight : 0.0, renorm_val : 0.0, error_rp : 0.00235023279674, error_yaw : 0.00180114735849}
HWSTATUS {Vcc : 5000, I2Cerr : 0}
SYSTEM_TIME {time_unix_usec : 1541471159699221, time_boot_ms : 32485335}
TERRAIN_REPORT {lat : -353632563, lon : 1491652315, spacing : 100, terrain_height : 583.887329102, current_height : 0.0015869140625, pending : 0, loaded : 504}
BATTERY_STATUS {id : 0, battery_function : 0, type : 0, temperature : 32767, voltages : [12587, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535], current_battery : 0, current_consumed : 144186, energy_consumed : 63240, battery_remaining : 83, time_remaining : 0, charge_state : 0}
```
