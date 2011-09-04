import math, logging

class InvalidIKInput(Exception):
        """
        Exception Class for Inverse kinematics methods of hexapod.Limb class 
        """
        def __init__(self, value):
            self.value = value
        def __str__(self):
            return repr(self.value)

class JointLimit(Exception):
        """
        Exception Class for when joint limits are exeeded
        """
        def __init__(self, value):
            self.value = value
        def __str__(self):
            return repr(self.value)

class Limb:
    """
    Class that represents the limb of a the hexapod robot.
    performs (inverse) kinematic calculations.
    """
    
    def __init__(self, 
                 name, 
                 controller, 
                 coxaAddr, 
                 femurAddr, 
                 tibiaAddr, 
                 coxaOffset = 90, 
                 femurOffset = 90, 
                 tibiaOffset = 90,
                 
                 coxaMirror = False,
                 femurMirror = False,
                 tibiaMirror = False,
                ):
        self.name = name
        self.controller = controller
        #indeces for joint angles in controller position list
        self.coxaAddr = coxaAddr
        self.femurAddr = femurAddr
        self.tibiaAddr = tibiaAddr
        #angles to use when zeroing joints
        self.coxaOffset = coxaOffset
        self.femurOffset = femurOffset
        self.tibiaOffset = tibiaOffset
        #vars to hold current joint angles
        self.coxaAngle = 0
        self.femurAngle = 0
        self.tibiaAngle = 0
        #mirror flags. If set the joint angles are inverse eg. 80 is 100 relative to the limb frame and positive direction
        self.coxaMirror=coxaMirror
        self.femurMirror=femurMirror
        self.tibiaMirror=tibiaMirror


        self.logger = logging.getLogger(name)
        self.logger.info('Limb Initialized')

    def place(self, x,y,z,l1=11.5,l2=45,l3=70):
        """
        Method that places the leg limb at x,y,z coordinates relative to limb
        frame
        """
        self.logger.info('Calculating IK for leg tip position: (%d,%d,%d)'%(x,y,z))

        #length from inner joint to tip of leg seen from above
        a=math.sqrt(x**2+y**2)
        #self.logger.info('a=%f'%a)
        
        if a>l1+l2+l3 or a<0:
            raise InvalidIKInput('Length from coxa joint to end point is impossible!')

        #Length from the middle joint to the tip of the leg seen from the side
        b=math.sqrt((a-l1)**2+z**2)
        #self.logger.info('b=%f'%b)
        if b>l2+l3 or b<0:
            raise InvalidIKInput('Length from femur to tibia joints is impossible!')

        #Angle between the line from the middle joint to the tip and the x-y plane
        IKA1=math.atan2(z,a-l1)
        #self.logger.info('IKA1=%f'%IKA1)

        #angle between the line from the middle joint to the tip and the middle bone
        try:
            IKA2=math.acos((-l3**2+l2**2+b**2)/(2*l2*b))
            #self.logger.info('IKA2=%f'%IKA2)
        except:
            raise InvalidIKInput('Could not calculate IKA2 angle!')

        #Inner joint angle in degrees.
        #If x zero this means end point in coxa axis no need to move.
        #might change this to try and center the leg?
        if x == 0:
            pass
        #if x is less than zero the endpoint is either under or over the body. to avoid impossibl angle add 180 degrees.
        elif x < 0:
            coxaAngle=math.atan2(y,x)*(180/math.pi)+ 180
        #normal operation
        else:
            coxaAngle=math.atan2(y,x)*(180/math.pi)

        self.logger.info('Coxa Angle: %f'%coxaAngle)

        #Middle joint angle in degrees.
        femurAngle=((IKA1+IKA2)*(180/math.pi))
        self.logger.info('Femur Angle: %f'%femurAngle)

        #Outer joint angle in degrees.
        try:
            tibiaAngle= math.acos((b**2-l3**2-l2**2)/(-2*l3*l2))*(180/math.pi) - 180
            self.logger.info('Tibia Angle: %f'%tibiaAngle)
        except:
            raise InvalidIKInput('Could not calculate tibia joint angle!')

        #Finally set joint angles
        self.setJoints(coxaAngle, femurAngle, tibiaAngle)

    def setJoints(self, coxaAngle, femurAngle, tibiaAngle):
        """
        Method that sets the limb joints to the angles specified in self.xxxxxAngle vars
        """
        self.setCoxaJoint(coxaAngle)
        self.setFemurJoint(femurAngle)
        self.setTibiaJoint(tibiaAngle)

    def setCoxaJoint(self,angle):
        """
        method thats sets the coxa joint angle
        """
        if not( type(angle) is float or type(angle) is int):
            raise TypeError('Input parameter must be integer!')
        if self.coxaMirror:
            self.coxaAngle = self.mirrorAngle(angle)
        else:
            self.coxaAngle = angle
        #send angle to controller via controller internal positioning list. Remember to add joint offset.
        self.controller.Positions[self.coxaAddr] = self.coxaAngle + self.coxaOffset

    def setFemurJoint(self,angle):
        """
        method thats sets the femur joint angle
        """
        if not( type(angle) is float or type(angle) is int):
            raise TypeError('Input parameter must be integer!')
        if self.femurMirror:
            self.femurAngle = self.mirrorAngle(angle)
        else:
            self.femurAngle = angle

        self.controller.Positions[self.femurAddr] = self.femurAngle + self.femurOffset

    def setTibiaJoint(self,angle):
        """
        method thats sets the tibia joint angle
        """
        if not( type(angle) is float or type(angle) is int):
            raise TypeError('Input parameter must be integer!')
        if self.tibiaMirror:
            self.tibiaAngle =self.mirrorAngle(angle)
        else:
            self.tibiaAngle = angle
        
        self.controller.Positions[self.tibiaAddr] = self.tibiaAngle + self.tibiaOffset

    def mirrorAngle(self,angle):
        """
        method that "mirrors" an angle. eg. 110 becomes 70, 250 becomes 290 and 180 becomes 0
        """
        if angle < 180:
            return 180 - angle
        else:
            return 360 - (angle - 180)


    def zeroJoints(self):
        """
        method that returns all joints to their center position
        """
        self.logger.info('Zeroing Joints.')

        self.setCoxaJoint(0) 
        self.setFemurJoint(0)
        self.setTibiaJoint(0)
