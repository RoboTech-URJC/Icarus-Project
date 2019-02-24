#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from pymavlink import mavutil
import sys

class Drone:
    def __init__(self, port):
        '''
        Port is where drone is listening. As UDP or TCP as Serial
        '''
        #To create la conection:
        self.ardupilot = mavutil.mavlink_connection(port, baud=57600) #Open the connection

        #Parameters:
        self.ardupilot_sys_id = self.ardupilot.target_system
        self.ardupilot_component_id = self.ardupilot.target_component

        self.wait_heartbeat() #prints the initial parameters of the drone

        self.ardupilot.mav.heartbeat_send(
        2, # type
        3, # autopilot
        81, # base_mode
        0, # custom_mode
        3, # system_status
        3  # mavlink_version
        )


        self.change_mode('STABILIZE')

    #HERE STARTS THE MAIN METHODS

    def arm_disarm(self, state, failsafe):
        '''
        arm or disarm the drone depending of the parameter state:
        if state = 1, then arm drone;
        if state = 0, then disarm drone;
        if state != 0 and state != 1, return a message of error
        failsafe == 1 serves to disarm the drone on flight
        '''
        def is_correct(n):
            return n == 0 or n == 1

        if(is_correct(state) and is_correct(failsafe)):
            '''
            while(True):
                msg = None
                while(msg == None):
                    msg = self.ardupilot.recv()

                print(msg)
            '''
            self.ardupilot.mav.command_long_send(self.ardupilot_sys_id,
                                            self.ardupilot_component_id,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                            0, #confirmation
                                            state, failsafe, 0, 0, 0, 0, 0)
            #self.ardupilot.write(buf)

            #self.ardupilot.arducopter_arm()

            if(state == 1):
                #wait to arm
                self.ardupilot.motors_armed_wait()
                #self.ardupilot.write(buf)
            elif(state == 0):
                #wait to disarm
                self.ardupilot.motors_disarmed_wait()

        else:
            print("[USAGE ERROR] state value is 1 or 0 for arm/disarm the drone")
            print("failsafe value is 1/0 for disarm or not disarm the drone on flight")

    def takeoff(self, altitude):
        '''
        Take off the drone to a specific altitude
        '''

        def wait_to_takeoff():

            error_range = 0.08
            alt = self.get_alt()
            while(alt < altitude - error_range):
                alt = self.get_alt()
                print(alt)

        #self.change_mode('AUTO') # set mode flight

        self.ardupilot.mav.command_long_send(self.ardupilot_sys_id,
                                        self.ardupilot_component_id,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                        0, #confirmation
                                        0, 0, 0, 0, 0, 0, altitude)

        '''
        #self.ardupilot.write(buf)
        while(True):
            msg = None
            while(msg == None):
                msg = self.ardupilot.recv_match()
            print(msg)
        #time.sleep(2)
        '''
        wait_to_takeoff() #Block the method while drone is taking off

    def land(self):
        '''
        Land the drone
        '''

        def wait_to_land():

            error_range = 0.08

            alt = self.get_alt()
            while(alt > error_range):
                alt = self.get_alt()

        self.change_mode('LAND')
        wait_to_land() #Block the method while drone is landing

    #HERE STARTS THE SECONDARY METHODS

    def get_alt(self):
        '''
        Returns the altitude in meters
        '''

        param = self.ardupilot.recv_match(type='GLOBAL_POSITION_INT', blocking = True) #receive all parameters
        alt = param.relative_alt * 0.001
        return alt

    def change_mode(self, mode):
        '''
        Change the flight mode
        '''

        self.ardupilot.set_mode_apm(mode) # set mode flight
        print("Mode changed to " + mode)

    def wait_heartbeat(self):
        '''
        wait for a heartbeat so we know the target system IDs
        '''

        print("Waiting for APM heartbeat")
        msg = self.ardupilot.recv_match(type='HEARTBEAT', blocking=True)
        print(msg)
        #self.ardupilot.pre_message()
        #print(self.ardupilot.location())
        print("Heartbeat from APM (system %u component %u)" % (self.ardupilot.target_system,
                                                                    self.ardupilot.target_component))
