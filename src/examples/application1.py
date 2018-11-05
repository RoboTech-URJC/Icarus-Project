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
#creamos el objeto de la clase Drone
icarus = di.Drone(port)

#armar el drone
icarus.arm_disarm(1, 0)

#Despegar el drone
icarus.takeoff(20)
#time.sleep(10)

#aterrizar el drone
icarus.land()

#desarmar el drone
icarus.arm_disarm(0, 0)
