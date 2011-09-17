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
    def __init__(self):
        self.logger = logging.getLogger('Joystick')
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
        self.body_z = 0.0
        self.yaw = 0.0
        
        
    def handle_js_event(self, event):
        '''
        Method to handle pygame events from the Joystick
        '''
        self.logger.info('Got pygame event %s', event.dict)
        if event.type == pygame.JOYAXISMOTION:
            #build 2D vector of input
            if event.dict['axis'] == 0:
                self.x = -float(event.dict['value'])
                self.logger.info('Translation: (%f, %f)', self.x, self.y)
            
            if event.dict['axis'] == 1:
                self.y = -event.dict['value']
                self.logger.info('Translation: (%f, %f)', self.x, self.y)
            
            if event.dict['axis'] == 2:
                self.yaw = event.dict['value']
                self.logger.info('Yaw: %f',self.yaw)
                
                
            
            
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

                