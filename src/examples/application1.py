'''
Application that uses the driver 'driver-icarus'
1) Arm the Drone
2) Take off the Drone
3) Land the Drone
4) Disarm the Drone
'''
import driver_icarus as di
import time

port = 'udpin:localhost:14551'

#Create object of Drone class
icarus = di.Drone(port)

#Arm the drone
icarus.arm_disarm(1, 0)

#Take off the drone
icarus.takeoff(20)
#time.sleep(10)

#Land the drone
icarus.land()

#Disarm the drone
icarus.arm_disarm(0, 0)
