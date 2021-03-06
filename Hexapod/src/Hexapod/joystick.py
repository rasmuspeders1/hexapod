'''
Created on Sep 10, 2011

@author: rasmus
'''

import pygame
import numpy
import logging
import threading

class RobotInput(threading.Thread):
    '''
    Class to handle joystick input and translate it to something more useful 
    for robot control.
    '''
    def __init__(self, threshold=0):
        self.logger = logging.getLogger('Joystick')
        self.threshold = threshold
        pygame.init()
        self.js_list = []
        if pygame.joystick.get_count() == 0:
            self.logger.error('Found no joystick. Please connect joystick and try again.')
        else:
            for i in range(pygame.joystick.get_count()):
                self.js_list.append(pygame.joystick.Joystick(i))
                self.js_list[i].init()
                logging.info('Found Joystick: %s ', self.js_list[i].get_name())

        self.stop_event = threading.Event()
        self.stop_event.clear()

        threading.Thread.__init__(self)
        self.x = 0.0
        self.y = 0.0

        # Body relative offset
        self.body_x = 0.0
        self.body_y = 0.0
        self.body_z = 0.0
        self.body_offset_lock = False


        # Robot yaw angle
        self.yaw = 0.0

        # Gripper
        self.grip = 0.0
        self.gripper_angle_delta = 0.0
        self.gripper_lock = False


    def handle_js_event(self, event):
        '''
        Method to handle pygame events from the Joystick
        '''
        #self.logger.info('Got pygame event %s', event.dict)
        if event.type == pygame.JOYBUTTONDOWN:
            # Cross
            if event.dict['button'] == 14:
                self.logger.debug('Locked body Z offset to: %d', self.body_z)
                self.body_offset_lock = not self.body_offset_lock

            # Square
            if event.dict['button'] == 15:
                self.logger.debug('Locked gripper to: %d', self.grip)
                self.gripper_lock = not self.gripper_lock

            # L1
            if event.dict['button'] == 10:
                self.logger.debug('Turning gripper left')
                self.gripper_angle_delta += 1.0

            # R1
            if event.dict['button'] == 11:
                self.logger.debug('Turning gripper right')
                self.gripper_angle_delta -= 1.0

        if event.type == pygame.JOYBUTTONUP:
            # L1
            if event.dict['button'] == 10:
                self.logger.debug('No longer turning gripper left')
                self.gripper_angle_delta -= 1.0

            if event.dict['button'] == 11:
                self.logger.debug('No longer turning gripper right')
                self.gripper_angle_delta += 1.0

        if event.type == pygame.JOYAXISMOTION:
            if event.dict['axis'] == 0:
                new_x = -float(event.dict['value'])
                if abs(new_x - self.x) > self.threshold:
                    self.x = new_x
                    self.logger.info('Translation: (%f, %f)', self.x, self.y)

            if event.dict['axis'] == 1:
                new_y = -float(event.dict['value'])
                if abs(new_y - self.y) > self.threshold:
                    self.y = new_y
                    self.logger.info('Translation: (%f, %f)', self.x, self.y)

            if event.dict['axis'] == 2:
                new_yaw = event.dict['value']
                if abs(new_yaw - self.yaw) > self.threshold:
                    self.yaw = new_yaw
                    self.logger.info('Yaw: %f', self.yaw)

            # Body Z offset
            if event.dict['axis'] == 13:
                if not self.body_offset_lock:
                    new_body_z = event.dict['value']
                    if abs(new_body_z - self.body_z) > self.threshold:
                        self.body_z = new_body_z
                        self.logger.info('Body Z offset: %f', self.body_z)

            #Gripper grap
            if event.dict['axis'] == 12:
                if not self.gripper_lock:
                    new_grip = event.dict['value']
                    if abs(new_grip - self.grip) > self.threshold:
                        self.grip = new_grip
                        self.logger.info('Grip: %f', self.grip)

    def run(self):
        '''
        run method of the joystick input thread
        '''
        while not self.stop_event.isSet():
            event = pygame.event.wait()
            if not event is None:
                self.handle_js_event(event)

    def stop(self):
        self.logger.info('Stopping')
        self.stop_event.set()
        self.join()


