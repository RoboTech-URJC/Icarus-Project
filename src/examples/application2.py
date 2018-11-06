'''
Application that uses the driver 'driver-icarus'
1) Arm the Drone
2) Take off the Drone
3) Land the Drone
4) Disarm the Drone
Do this in loop
'''
import driver_icarus as di
import time

port = 'udpin:localhost:14551'
#Create object of Drone class
icarus = di.Drone(port)
try:
    while(1):

        #Arm the drone
        icarus.arm_disarm(1, 0)
        #time.sleep(5)

        #Take off the drone
        icarus.takeoff(15)

        #Land the drone
        icarus.land()

        #Disarm the drone
        icarus.arm_disarm(0, 0)
except KeyboardInterrupt:
    icarus.land()
