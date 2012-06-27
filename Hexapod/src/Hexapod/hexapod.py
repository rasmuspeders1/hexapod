

import sys
from . import maestro
from .kinematics import *
from .gaitengine import *


class Hexapod:
    """
    Class that represents a hexapod robot.
    """
    def __init__(self):
        # set up logging to file - see previous section for more details
        logging.basicConfig(level=logging.DEBUG,
                                          format='%(asctime)s-%(filename)s#%(lineno)d %(name)s %(levelname)s: %(message)s',
                                          filename='hexapod.log',
                                          filemode='w')
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

        self.hexapod_logger = logging.getLogger('Hexapod')

        self.hexapod_logger.info('Hexapod Initializing...')

        #First create body. This is needed for all limbs
        self.body = Body('Hexapod Body')

        #Dictionary to hold all servo positions for maestro controller
        self.servo_positions = {}
        #Create Robot Links and Limbs
        self.limbs = {}

        #Left Front limb
        lf_link0 = Link(0, 55.75, 19, 30, 'Left Front Link0', None, 0)
        lf_link1 = Link(90, 11.50, 0, 0, 'Left Front Link1', 14, 75)
        lf_link2 = Link(180, 45.00, 0, 0, 'Left Front Link2', 13, 118, True)
        lf_link3 = Link(0, 70.00, 0, 0, 'Left Front Link3', 12, 165)

        self.lf_limb = Limb(self.body, lf_link0, lf_link1, lf_link2, lf_link3, 'Left Front Limb'.ljust(18))
        self.limbs['lf'] = self.lf_limb

        #Left Middle limb
        lm_link0 = Link(0, 35.75, 19, 90, 'Left Middle Link0', None, 0)
        lm_link1 = Link(90, 11.50, 0, 0, 'Left Middle Link1', 8, 85)
        lm_link2 = Link(180, 45.00, 0, 0, 'Left Middle Link2', 7, 120, True)
        lm_link3 = Link(0, 70.00, 0, 0, 'Left Middle Link3', 6, 175)

        self.lmLimb = Limb(self.body, lm_link0, lm_link1, lm_link2, lm_link3, 'Left Middle Limb'.ljust(18))
        self.limbs['lm'] = self.lmLimb

        #Left Back limb
        lb_link0 = Link(0, 55.75, 19, 150, 'Left Back Link0', None, 0)
        lb_link1 = Link(90, 11.50, 0, 0, 'Left Back Link1', 2, 105)
        lb_link2 = Link(180, 45.00, 0, 0, 'Left Back Link2', 1, 90, True)
        lb_link3 = Link(0, 70.00, 0, 0, 'Left Back Link3', 0, 180)

        self.lb_limb = Limb(self.body, lb_link0, lb_link1, lb_link2, lb_link3, 'Left Back Limb'.ljust(18))
        self.limbs['lb'] = self.lb_limb

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
        rb_link0 = Link(0, 55.75, 19, -150, 'Right Back Link0', None, 0)
        rb_link1 = Link(-90, 11.50, 0, 0, 'Right Back Link1', 5, 65)
        rb_link2 = Link(180, 45.00, 0, 0, 'Right Back Link2', 4, 90)
        rb_link3 = Link(0, 70.00, 0, 0, 'Right Back Link3', 3, 10, True)

        self.rb_limb = Limb(self.body, rb_link0, rb_link1, rb_link2, rb_link3, 'Right Back Limb'.ljust(18))
        self.limbs['rb'] = self.rb_limb

        self.gripper_angle_servo_addr = 18
        self.gripper_servo_addr = 19
        self.gripper_angle_offset = 81.0
        self.grip_offset = 90.0
        self.gripper_angle = 0.0
        self.grip = 0.0

        #Init maestro control
        #First init servo position object to input to maestro controller
        self.controller = maestro.Maestro(get_pos=self.get_positions)

        #Update center offsets in controller
        for limb in self.limbs.values():
            for link in limb.links:
                if link.servoAddr is not None:
                    self.controller.centerOffsets[link.servoAddr] = link.centerOffset

        self.gait_engine = GaitEngine(self)

        self.hexapod_logger.info('Hexapod Initialized.')

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

    def set_gripper_angle(self, angle):
        '''Method that sets the angle of the gripper'''
        self.gripper_angle = max(-45.0, min(45, float(angle)))

    def set_grip(self, grip):
        self.grip = max(-10.0, min(70, float(grip)))

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
        feet_positions = self.gait_engine.get_feet_positions()

        for foot, limb in self.limbs.items():
            limb.place(feet_positions[foot])
            self.servo_positions[limb.link1.servoAddr] = limb.link1.theta
            self.servo_positions[limb.link2.servoAddr] = limb.link2.theta
            self.servo_positions[limb.link3.servoAddr] = limb.link3.theta

        self.servo_positions[self.gripper_angle_servo_addr] = self.gripper_angle + self.gripper_angle_offset
        self.servo_positions[self.gripper_servo_addr] = self.grip + self.grip_offset

        return self.servo_positions
