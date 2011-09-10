"""
Python module that h:elps perform kinematic and inverse kinematic operations
for a hexapod robot.
"""
import logging, numpy
import math
from math import radians as rad
from math import cos as cos
from math import sin as sin

def get_t_matrix(translation, rotation):
    """
    method that returns a generic transformation matrix 
    from a 3-dimensional position and rotation
    """
    roll = float(rotation[0])
    pitch = float(rotation[1])
    yaw = float(rotation[2])
    
    TMatrix11 = cos(rad(yaw))*cos(rad(pitch))
    TMatrix21 = sin(rad(yaw))*cos(rad(pitch))
    TMatrix31 = -sin(rad(pitch))
    TMatrix41 = 0

    TMatrix12 = -cos(rad(roll))*sin(rad(yaw)) + cos(rad(yaw))*sin(rad(roll))*sin(rad(pitch))
    TMatrix22 = cos(rad(roll))*cos(rad(yaw)) + sin(rad(roll))*sin(rad(pitch))*sin(rad(yaw))
    TMatrix32 = cos(rad(pitch))*sin(rad(roll))
    TMatrix42 = 0

    TMatrix13 = sin(rad(roll))*sin(rad(yaw)) + cos(rad(roll))*cos(rad(yaw))*sin(rad(pitch))
    TMatrix23 = -cos(rad(yaw))*sin(rad(roll)) + cos(rad(roll))*sin(rad(pitch))*sin(rad(yaw))
    TMatrix33 = cos(rad(roll))*cos(rad(pitch))
    TMatrix43 = 0

    TMatrix14 = translation[0]
    TMatrix24 = translation[1]
    TMatrix34 = translation[2]
    TMatrix44 = 1
                     
    return numpy.matrix([
                         [TMatrix11, TMatrix12, TMatrix13, TMatrix14],
                         [TMatrix21, TMatrix22, TMatrix23, TMatrix24],
                         [TMatrix31, TMatrix32, TMatrix33, TMatrix34],
                         [TMatrix41, TMatrix42, TMatrix43, TMatrix44]
                        ]
                       )
   
class InvalidIKInput(Exception):
    """
    Exception Class for Inverse kinematics methods of hexapod.Limb class 
    """
    def __init__(self, value):
        self.value = value
        Exception.__init__(self)

    def __str__(self):
        return repr(self.value)

class JointLimit(Exception):
    """
    Exception Class for when joint limits are exeeded
    """
    def __init__(self, value):
        self.value = value
        Exception.__init__(self)

    def __str__(self):
        return repr(self.value)

class Body:
    """
    Class that represents the body of the robot.
    Contains transformations matrices for body->global global->body coordinates.
    """
    def __init__(self, name='Hexapod'):
        self.name = name
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.z = 0

    def set_rotation(self, (roll, pitch, yaw)):
        """
        Method that sets the Robot Bodys rotational angles.
        This has effect on the rotational matrix and therefore 
        also the transformation and inverse tranformation matrices.
        """
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def set_pos(self, pos):
        """
        Method that sets the body translational coordinates relative to the global frame.
        """
        self.x = float(pos[0])
        self.y = float(pos[1])
        self.z = float(pos[2])

    def get_t_matrix(self):
        """
        method that returns the transformation matrix for the robot body
        This matrix transforms body coordinates the global coordinates
        """
        TMatrix11 = cos(rad(self.yaw))*cos(rad(self.pitch))
        TMatrix21 = sin(rad(self.yaw))*cos(rad(self.pitch))
        TMatrix31 = -sin(rad(self.pitch))
        TMatrix41 = 0

        TMatrix12 = -cos(rad(self.roll))*sin(rad(self.yaw)) + cos(rad(self.yaw))*sin(rad(self.roll))*sin(rad(self.pitch))
        TMatrix22 = cos(rad(self.roll))*cos(rad(self.yaw)) + sin(rad(self.roll))*sin(rad(self.pitch))*sin(rad(self.yaw))
        TMatrix32 = cos(rad(self.pitch))*sin(rad(self.roll))
        TMatrix42 = 0

        TMatrix13 = sin(rad(self.roll))*sin(rad(self.yaw)) + cos(rad(self.roll))*cos(rad(self.yaw))*sin(rad(self.pitch))
        TMatrix23 = -cos(rad(self.yaw))*sin(rad(self.roll)) + cos(rad(self.roll))*sin(rad(self.pitch))*sin(rad(self.yaw))
        TMatrix33 = cos(rad(self.roll))*cos(rad(self.pitch))
        TMatrix43 = 0

        TMatrix14 = self.x
        TMatrix24 = self.y
        TMatrix34 = self.z
        TMatrix44 = 1
                         
        return numpy.matrix([
                             [TMatrix11, TMatrix12, TMatrix13, TMatrix14],
                             [TMatrix21, TMatrix22, TMatrix23, TMatrix24],
                             [TMatrix31, TMatrix32, TMatrix33, TMatrix34],
                             [TMatrix41, TMatrix42, TMatrix43, TMatrix44]
                            ]
                           )
        
    def getRMatrix(self):
        TMatrix11 = cos(rad(self.yaw))*cos(rad(self.pitch))
        TMatrix21 = sin(rad(self.yaw))*cos(rad(self.pitch))
        TMatrix31 = -sin(rad(self.pitch))

        TMatrix12 = -cos(rad(self.roll))*sin(rad(self.yaw)) + cos(rad(self.yaw))*sin(rad(self.roll))*sin(rad(self.pitch))
        TMatrix22 = cos(rad(self.roll))*cos(rad(self.yaw)) + sin(rad(self.roll))*sin(rad(self.pitch))*sin(rad(self.yaw))
        TMatrix32 = cos(rad(self.pitch))*sin(rad(self.yaw))

        TMatrix13 = sin(rad(self.roll))*sin(rad(self.yaw)) + cos(rad(self.roll))*cos(rad(self.yaw))*sin(rad(self.pitch))
        TMatrix23 = -cos(rad(self.yaw))*sin(rad(self.roll)) + cos(rad(self.roll))*sin(rad(self.pitch))*sin(rad(self.yaw))
        TMatrix33 = cos(rad(self.roll))*cos(rad(self.pitch))

        return numpy.matrix([
                             [TMatrix11, TMatrix12, TMatrix13],
                             [TMatrix21, TMatrix22, TMatrix23],
                             [TMatrix31, TMatrix32, TMatrix33]
                            ]
                           )

    def getTranslationMatrix(self):
        return numpy.matrix([
                             [self.x],
                             [self.y],
                             [self.z],
                            ]
                           )

    def getInverseTMatrix(self):

        rMatrix = self.getRMatrix()
        
        translationMatrix = self.getTranslationMatrix()

        iTMatrix = rMatrix.transpose()
        
        matrix2  = - (rMatrix.transpose() * translationMatrix)
        
        iTMatrix = numpy.append(iTMatrix, matrix2, axis = 1)
        iTMatrix = numpy.append(iTMatrix, numpy.matrix([0,0,0,1]), axis = 0)

        return iTMatrix

class Link:
    """
    Class that represents a rotional link of the robot.
    """
    def __init__(self,
                  alpha,
                  a,
                  d,
                  theta,
                  name = 'Robot Link',
                  servoAddr = None,
                  centerOffset = 0,
                  mirrored = False
                ):
        
        """
        The Link class is instantiated with a set of 
        Denavit Hartenberg parameters, 
        describing the link and a name for the link.
        """
        self.alpha = float(alpha)
        self.a = float(a)
        self.d = float(d)
        self.theta = float(theta)
        self.name = name
        self.servoAddr = servoAddr
        self.centerOffset = centerOffset
        self.mirrored = mirrored

    def get_t_matrix(self):
        return numpy.matrix([
                             [ math.cos(math.radians(self.theta)), -math.sin(math.radians(self.theta)) * math.cos(math.radians(self.alpha)),  math.sin(math.radians(self.theta)) * math.sin(math.radians(self.alpha)), self.a * math.cos(math.radians(self.theta))],
                             [ math.sin(math.radians(self.theta)),  math.cos(math.radians(self.theta)) * math.cos(math.radians(self.alpha)), -math.cos(math.radians(self.theta)) * math.sin(math.radians(self.alpha)), self.a * math.sin(math.radians(self.theta))],
                             [                    0,                         math.sin(math.radians(self.alpha)),                         math.cos(math.radians(self.alpha)),                        self.d],
                             [                    0,                                            0,                                            0,                             1]
                            ]
                           )
    def getRMatrix(self):
        return numpy.matrix([
                             [ math.cos(math.radians(self.theta)), -math.sin(math.radians(self.theta)) * math.cos(math.radians(self.alpha)),  math.sin(math.radians(self.theta)) * math.sin(math.radians(self.alpha))],
                             [ math.sin(math.radians(self.theta)),  math.cos(math.radians(self.theta)) * math.cos(math.radians(self.alpha)), -math.cos(math.radians(self.theta)) * math.sin(math.radians(self.alpha))],
                             [                    0,                         math.sin(math.radians(self.alpha)),                         math.cos(math.radians(self.alpha))],
                            ]
                           )
    def getTranslationMatrix(self):
        return numpy.matrix([
                             [self.a * math.cos(math.radians(self.theta))],
                             [self.a * math.sin(math.radians(self.theta))],
                             [                                     self.d],
                            ]
                           )

    def getInverseTMatrix(self):

        rMatrix = self.getRMatrix()
        
        translationMatrix = self.getTranslationMatrix()

        iTMatrix = rMatrix.transpose()
        
        matrix2  = - (rMatrix.transpose() * translationMatrix)
        
        iTMatrix = numpy.append(iTMatrix, matrix2, axis = 1)
        iTMatrix = numpy.append(iTMatrix, numpy.matrix([0,0,0,1]), axis = 0)

        return iTMatrix


    def setTheta(self, theta):
        if self.mirrored:
            self.theta = -theta
        else:
            self.theta = theta


class Limb:
    """
    Class that represents a robot limb with 3 degrees of freedom and with rotational links. 
    """
    
    def __init__(self,
                 body,
                 link0,
                 link1,
                 link2,
                 link3,
                 name = 'Robot Limb'
                 ):
        """
        The robot limb class is instatiated with 4 link objects and a name.
        link0 represent the origin of the link relative to the body frame 
        link1-3 represents the rotational joints.
        the origin of the link3 frame is the endpoint of the limb.
        """
        self.body = body
        self.link0 = link0
        self.link1 = link1
        self.link2 = link2
        self.link3 = link3
        self.links=[self.link0, self.link1, self.link2, self.link3]
        self.name = name
        self.logger = logging.getLogger(name)
        
        
    
    
    def getEPPos(self):
        """
        method that returns a vector (numpy matrix) with the x,y,z coordinates of the limb endpoint in body coordinates.
        """
        return self.link0.getTMatrix() * self.link1.getTMatrix() * self.link2.getTMatrix() * self.link3.getTMatrix() * numpy.matrix([[0],[0],[0],[1]])

    def logEPPos(self):
        """
        Method that log the current Position of the limb endpoint in body coordinates
        """
        EPPos = self.getEPPos()
        self.logger.info('Enpoint Position: (%f, %f, %f)'%(EPPos[0], EPPos[1], EPPos[2]))


    def place(self, pos):
        """
        Method that calcultate the rotational joint angles for the leg endpoint to be placed in x,y,z coordinates relative to the body frame.
        Input coordinates must be given in milimeters relative to the body frame. Floats, integers and strings with either are acceptable input.
        """
        pos = numpy.append(pos, numpy.matrix([[1]]), axis=0)

        #log inverse kinematic operation
        self.logger.debug('Calculating joint angles for leg endpoint position: (%.2f,%.2f,%.2f)'%(pos[0],pos[1],pos[2]))
        
        #Input coordinates are given in body frame coordinates
        #Transform coordinates to link0 frame
        link0Coords = self.link0.getInverseTMatrix() * self.body.getInverseTMatrix () * pos
        
        self.logger.debug('Endpoint Positions in Link0 Coords: (%.2f,%.2f,%.2f)'%(link0Coords[0],link0Coords[1],link0Coords[2]))
        
        #length from link0 to link4
        a=math.sqrt(link0Coords[0]**2+link0Coords[1]**2)
        self.logger.debug('a=%f'%a)
        
        #Check that robot can reach point. eg. endpoint position  not without limb reach.
        if a > self.link1.a + self.link2.a + self.link3.a or a < 0:
            raise InvalidIKInput('Length from link0 to link3 is impossible!')


        #Length from link2 (femur) to the tip of the leg seen from the side
        b=math.sqrt((a-self.link1.a)**2 + link0Coords[2]**2)
        self.logger.debug('b=%f'%b)

        if b > self.link2.a + self.link3.a or b < 0:
            raise InvalidIKInput('Length from link2 to link3 is impossible!')

        #Angle between the line from the middle joint to the tip and the x-y plane
        IKA1=math.atan2(link0Coords[2], a - self.link1.a)
        self.logger.debug('IKA1=%f'%IKA1)

        #angle between the line from the middle joint to the tip and the middle bone
        try:
            IKA2=math.acos((-self.link3.a**2 + self.link2.a**2 + b**2) / (2 * self.link2.a * b))
            self.logger.debug('IKA2=%f'%IKA2)
        except:
            raise InvalidIKInput('Could not calculate IKA2 angle!')

        #Inner joint angle in degrees.
        #If x zero this means end point in coxa axis no need to move.
        #might change this to try and center the leg?
        if link0Coords[0] == 0:
            pass
        #if x is less than zero the endpoint is either under or over the body. to avoid impossible angle add 180 degrees to coxa angle (theta1).
        elif link0Coords[0] < 0:
            theta1=math.atan2(link0Coords[1],link0Coords[0]) * (180/math.pi) + 180
        #normal operation
        else:
            theta1=math.atan2(link0Coords[1],link0Coords[0]) * (180/math.pi)

        self.logger.debug('Coxa Angle (theta1): %f'%theta1)

        #Middle joint angle in degrees.
        theta2=((IKA1+IKA2)*(180/math.pi))
        self.logger.debug('Femur Angle (theta2): %f'%theta2)

        #Outer joint angle in degrees.
        try:
            theta3 = math.acos((b**2 - self.link3.a**2 - self.link2.a**2) / (-2 * self.link3.a * self.link2.a )) * (180 / math.pi) - 180
            self.logger.debug('Tibia Angle (theta3): %f'%theta3)
        except:
            raise InvalidIKInput('Could not calculate tibia joint angle (theta3)!')

        #Set rotational angles (thetas) in links
        self.link1.setTheta(theta1)
        self.link2.setTheta(theta2)
        self.link3.setTheta(theta3)

