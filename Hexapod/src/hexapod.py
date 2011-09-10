#! /usr/bin/env python

import logging
import time
import sys
import maestro
from kinematics import *
from gaitengine import *


class Hexapod:
    """
    Class that represents a hexapod robot.
    """
    def __init__(self):
        # set up logging to file - see previous section for more details
        logging.basicConfig(level = logging.DEBUG,
                                          format = '%(asctime)s-%(filename)s#%(lineno)d %(name)s %(levelname)s: %(message)s',
                                          filename = 'hexapod.log',
                                          filemode = 'w')
        #define console stream handler
        console = logging.StreamHandler(sys.stdout)
        #Set console loglevel
        console.setLevel(logging.INFO)
        # set a format which is simpler for console use
        formatter = logging.Formatter('%(filename)s#%(lineno)d %(name)s %(levelname)s: %(message)s')
        # tell the handler to use this format
        console.setFormatter(formatter)
        # add the handler to the root logger
        logging.getLogger('').addHandler(console)

        self.hexapodLogger = logging.getLogger('Hexapod')

        self.hexapodLogger.info('Hexapod Initializing...')

        
        #First create body. This is needed for all limbs
        self.body = Body('Hexapod Body')

        #Dictionary to hold all servo positions for maestro controller
        self.servo_positions = {}
        #Create Robot Links and Limbs
        self.limbs = {}

        #Left Front limb
        lfLink0 = Link(0, 55.75, 19, 30, 'Left Front Link0', None, 0)
        lfLink1 = Link(90, 11.50, 0, 0, 'Left Front Link1', 14, 75)
        lfLink2 = Link(180, 45.00, 0, 0, 'Left Front Link2', 13, 118, True)
        lfLink3 = Link(0, 70.00, 0, 0, 'Left Front Link3', 12, 165)

        self.lfLimb = Limb(self.body, lfLink0, lfLink1, lfLink2, lfLink3, 'Left Front Limb'.ljust(18))
        self.limbs['lf'] = self.lfLimb
        
        #Left Middle limb
        lmLink0 = Link(0, 35.75, 19, 90, 'Left Middle Link0', None, 0)
        lmLink1 = Link(90, 11.50, 0, 0, 'Left Middle Link1', 8, 85)
        lmLink2 = Link(180, 45.00, 0, 0, 'Left Middle Link2', 7, 120, True)
        lmLink3 = Link(0, 70.00, 0, 0, 'Left Middle Link3', 6, 175)

        self.lmLimb = Limb(self.body, lmLink0, lmLink1, lmLink2, lmLink3, 'Left Middle Limb'.ljust(18))
        self.limbs['lm'] = self.lmLimb
        
        #Left Back limb
        lbLink0 = Link(0, 55.75, 19, 150, 'Left Back Link0', None, 0)
        lbLink1 = Link(90, 11.50, 0, 0, 'Left Back Link1', 2, 105)
        lbLink2 = Link(180, 45.00, 0, 0, 'Left Back Link2', 1, 90, True)
        lbLink3 = Link(0, 70.00, 0, 0, 'Left Back Link3', 0, 180)

        self.lbLimb = Limb(self.body, lbLink0, lbLink1, lbLink2, lbLink3, 'Left Back Limb'.ljust(18))
        self.limbs['lb'] = self.lbLimb
        
        #Right Front limb
        rfLink0 = Link(0, 55.75, 19, -30, 'Right Front Link0', None, 0)
        rfLink1 = Link(-90, 11.50, 0, 0, 'Right Front Link1', 17, 115)
        rfLink2 = Link(180, 45.00, 0, 0, 'Right Front Link2', 16, 90)
        rfLink3 = Link(0, 70.00, 0, 0, 'Right Front Link3', 15, 10, True)

        self.rfLimb = Limb(self.body, rfLink0, rfLink1, rfLink2, rfLink3, 'Right Front Limb'.ljust(18))
        self.limbs['rf'] = self.rfLimb
        
        #Right Middle limb
        rmLink0 = Link(0, 35.75, 19, -90, 'Right Middle Link0', None, 0)
        rmLink1 = Link(-90, 11.50, 0, 0, 'Right Middle Link1', 11, 95)
        rmLink2 = Link(180, 45.00, 0, 0, 'Right Middle Link2', 10, 90)
        rmLink3 = Link(0, 70.00, 0, 0, 'Right Middle Link3', 9, 10, True)

        self.rmLimb = Limb(self.body, rmLink0, rmLink1, rmLink2, rmLink3, 'Right Middle Limb'.ljust(18))
        self.limbs['rm'] = self.rmLimb
        
        #Right Back limb
        rbLink0 = Link(0, 55.75, 19, -150, 'Right Back Link0', None, 0)
        rbLink1 = Link(-90, 11.50, 0, 0, 'Right Back Link1', 5, 65)
        rbLink2 = Link(180, 45.00, 0, 0, 'Right Back Link2', 4, 90)
        rbLink3 = Link(0, 70.00, 0, 0, 'Right Back Link3', 3, 10, True)

        self.rbLimb = Limb(self.body, rbLink0, rbLink1, rbLink2, rbLink3, 'Right Back Limb'.ljust(18))
        self.limbs['rb'] = self.rbLimb

        
        #Init maestro control
        #First init servo position object to input to maestro controller
        self.controller = maestro.Maestro(get_pos = self.get_positions)
        #set as daemon. this means the controller thread dies with main python
        self.controller.daemon = True
           
        #Update center offsets in controller
        for limb in self.limbs.values():
            for link in limb.links:
                if link.servoAddr is not None:
                    self.controller.centerOffsets[link.servoAddr] = link.centerOffset
        

        self.gait_engine = GaitEngine()        

        self.hexapodLogger.info('Hexapod Initialized.')

    def __del__(self):
        self.controller.stop()
        
    def stop(self):
        """
        method that stops the hexapod
        """
        self.controller.stop()
        
    def start(self):
        """
        method to start the servo controller
        This will initiate polling of the get_feet_positions method, 
        so this must return valid positions when start() is called
        """
        self.controller.start()
        while not self.controller.started:
            time.sleep(0.1)
            
    def get_positions(self):
        """
        Method that returns the servo positions in the correct format for the controller object.
        """
        
        #update gait engine every controller period
        self.gait_engine.update()
        #place body
        self.body.set_pos(self.gait_engine.get_body_position())
        self.body.set_rotation(self.gait_engine.get_body_rotation())
        
        #update positions with values from gait engine
        #TODO: UPDATE POSITIONS
        feet_positions = self.gait_engine.get_feet_positions()
        
        for foot, limb in self.limbs.items():
            limb.place(feet_positions[foot])
            self.servo_positions[limb.link1.servoAddr] = limb.link1.theta
            self.servo_positions[limb.link2.servoAddr] = limb.link2.theta
            self.servo_positions[limb.link3.servoAddr] = limb.link3.theta
            
        return self.servo_positions
        
        

#define script behaviour if run as main
if __name__ == '__main__':
    hexapod = Hexapod()

    #Controller = HexController.HexController()
    #Controller.start()
    print('Press "CTRL-C" to exit.\n')
    hexapod.controller.go_home_all()
    
    try:
        hexapod.start()
        raw_input()
        hexapod.stop()
        
    except KeyboardInterrupt:
        logging.info('Received Keyboard Interrupt. Exiting.')
        hexapod.stop()
