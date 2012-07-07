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

    TMatrix11 = cos(rad(yaw)) * cos(rad(pitch))
    TMatrix21 = sin(rad(yaw)) * cos(rad(pitch))
    TMatrix31 = -sin(rad(pitch))
    TMatrix41 = 0

    TMatrix12 = -cos(rad(roll)) * sin(rad(yaw)) + cos(rad(yaw)) * sin(rad(roll)) * sin(rad(pitch))
    TMatrix22 = cos(rad(roll)) * cos(rad(yaw)) + sin(rad(roll)) * sin(rad(pitch)) * sin(rad(yaw))
    TMatrix32 = cos(rad(pitch)) * sin(rad(roll))
    TMatrix42 = 0

    TMatrix13 = sin(rad(roll)) * sin(rad(yaw)) + cos(rad(roll)) * cos(rad(yaw)) * sin(rad(pitch))
    TMatrix23 = -cos(rad(yaw)) * sin(rad(roll)) + cos(rad(roll)) * sin(rad(pitch)) * sin(rad(yaw))
    TMatrix33 = cos(rad(roll)) * cos(rad(pitch))
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
        cos_roll = cos(rad(self.roll))
        cos_pitch = cos(rad(self.pitch))
        cos_yaw = cos(rad(self.yaw))
        
        sin_roll = sin(rad(self.roll))
        sin_pitch = sin(rad(self.pitch))
        sin_yaw = sin(rad(self.yaw))
        
        TMatrix11 = cos_yaw * cos_pitch
        TMatrix21 = sin_yaw * cos_pitch
        TMatrix31 = -sin_pitch
        TMatrix41 = 0

        TMatrix12 = -cos_roll * sin_yaw + cos_yaw * sin_roll * sin_pitch
        TMatrix22 = cos_roll * cos_yaw + sin_roll * sin_pitch * sin_yaw
        TMatrix32 = cos_pitch * sin_roll
        TMatrix42 = 0

        TMatrix13 = sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch
        TMatrix23 = -cos_yaw * sin_roll + cos_roll * sin_pitch * sin_yaw
        TMatrix33 = cos_roll * cos_pitch
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
        cos_roll = cos(rad(self.roll))
        cos_pitch = cos(rad(self.pitch))
        cos_yaw = cos(rad(self.yaw))
        
        sin_roll = sin(rad(self.roll))
        sin_pitch = sin(rad(self.pitch))
        sin_yaw = sin(rad(self.yaw))
        
        TMatrix11 = cos_yaw * cos_pitch
        TMatrix21 = sin_yaw * cos_pitch
        TMatrix31 = -sin_pitch
        
        TMatrix12 = -cos_roll * sin_yaw + cos_yaw * sin_roll * sin_pitch
        TMatrix22 = cos_roll * cos_yaw + sin_roll * sin_pitch * sin_yaw
        TMatrix32 = cos_pitch * sin_roll

        TMatrix13 = sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch
        TMatrix23 = -cos_yaw * sin_roll + cos_roll * sin_pitch * sin_yaw
        TMatrix33 = cos_roll * cos_pitch

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

    def get_i_t_matrix(self):

        rMatrix = self.getRMatrix()

        translationMatrix = self.getTranslationMatrix()

        iTMatrix = rMatrix.transpose()

        matrix2 = -(rMatrix.transpose() * translationMatrix)

        iTMatrix = numpy.append(iTMatrix, matrix2, axis=1)
        iTMatrix = numpy.append(iTMatrix, numpy.matrix([0, 0, 0, 1]), axis=0)

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
                  name='Robot Link',
                  servoAddr=None,
                  centerOffset=0,
                  mirrored=False
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
        cos_theta = math.cos(math.radians(self.theta))
        cos_alpha = math.cos(math.radians(self.alpha))
        sin_theta = math.sin(math.radians(self.theta))
        sin_alpha = math.sin(math.radians(self.alpha))

        return numpy.matrix([
                             [ cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, self.a * cos_theta],
                             [ sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, self.a * sin_theta],
                             [                    0, sin_alpha, cos_alpha, self.d],
                             [                    0, 0, 0, 1]
                            ]
                           )
    def getRMatrix(self):
        cos_theta = math.cos(math.radians(self.theta))
        cos_alpha = math.cos(math.radians(self.alpha))
        sin_theta = math.sin(math.radians(self.theta))
        sin_alpha = math.sin(math.radians(self.alpha))
        return numpy.matrix([
                             [ cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha],
                             [ sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha],
                             [                    0, sin_alpha, cos_alpha],
                            ]
                           )
    def getTranslationMatrix(self):
        cos_theta = math.cos(math.radians(self.theta))
        sin_theta = math.sin(math.radians(self.theta))

        return numpy.matrix([
                             [self.a * cos_theta],
                             [self.a * sin_theta],
                             [         self.d],
                            ]
                           )

    def get_i_t_matrix(self):

        rMatrix = self.getRMatrix()

        translationMatrix = self.getTranslationMatrix()

        iTMatrix = rMatrix.transpose()

        matrix2 = -(rMatrix.transpose() * translationMatrix)

        iTMatrix = numpy.append(iTMatrix, matrix2, axis=1)
        iTMatrix = numpy.append(iTMatrix, numpy.matrix([0, 0, 0, 1]), axis=0)

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
                 name='Robot Limb'
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
        self.links = [self.link0, self.link1, self.link2, self.link3]
        self.name = name
        self.logger = logging.getLogger(name)

    def getEPPos(self):
        """
        method that returns a vector (numpy matrix) with the x,y,z coordinates of the limb endpoint in body coordinates.
        """
        return self.link0.getTMatrix() * self.link1.getTMatrix() * self.link2.getTMatrix() * self.link3.getTMatrix() * numpy.matrix([[0], [0], [0], [1]])

    def logEPPos(self):
        """
        Method that log the current Position of the limb endpoint in body coordinates
        """
        EPPos = self.getEPPos()
        self.logger.info('Enpoint Position: (%f, %f, %f)' % (EPPos[0], EPPos[1], EPPos[2]))


    def place(self, pos):
        """
        Method that calcultate the rotational joint angles for the leg endpoint to be placed in x,y,z coordinates relative to the body frame.
        Input coordinates must be given in milimeters relative to the body frame. Floats, integers and strings with either are acceptable input.
        """
        pos = numpy.append(pos, numpy.matrix([[1]]), axis=0)

        #log inverse kinematic operation
        self.logger.debug('Calculating joint angles for leg endpoint position: (%.2f,%.2f,%.2f)' % (pos[0], pos[1], pos[2]))

        #Input coordinates are given in body frame coordinates
        #Transform coordinates to link0 frame
        link0Coords = self.link0.get_i_t_matrix() * self.body.get_i_t_matrix () * pos

        self.logger.debug('Endpoint Positions in Link0 Coords: (%.2f,%.2f,%.2f)' % (link0Coords[0], link0Coords[1], link0Coords[2]))

        #length from link0 to link4
        a = math.sqrt(link0Coords[0] ** 2 + link0Coords[1] ** 2)
        self.logger.debug('a=%f' % a)

        #Check that robot can reach point. eg. endpoint position  not without limb reach.
        if a > self.link1.a + self.link2.a + self.link3.a or a < 0:
            raise InvalidIKInput('Length from link0 to link3 is impossible!')


        #Length from link2 (femur) to the tip of the leg seen from the side
        b = math.sqrt((a - self.link1.a) ** 2 + link0Coords[2] ** 2)
        self.logger.debug('b=%f' % b)

        if b > self.link2.a + self.link3.a or b < 0:
            raise InvalidIKInput('Length from link2 to link3 is impossible!')

        #Angle between the line from the middle joint to the tip and the x-y plane
        IKA1 = math.atan2(link0Coords[2], a - self.link1.a)
        self.logger.debug('IKA1=%f' % IKA1)

        #angle between the line from the middle joint to the tip and the middle bone
        try:
            IKA2 = math.acos((-self.link3.a ** 2 + self.link2.a ** 2 + b ** 2) / (2 * self.link2.a * b))
            self.logger.debug('IKA2=%f' % IKA2)
        except:
            raise InvalidIKInput('Could not calculate IKA2 angle!')

        #Inner joint angle in degrees.
        #If x zero this means end point in coxa axis no need to move.
        #might change this to try and center the leg?
        if link0Coords[0] == 0:
            pass
        #if x is less than zero the endpoint is either under or over the body. to avoid impossible angle add 180 degrees to coxa angle (theta1).
        elif link0Coords[0] < 0:
            theta1 = math.atan2(link0Coords[1], link0Coords[0]) * (180 / math.pi) + 180
        #normal operation
        else:
            theta1 = math.atan2(link0Coords[1], link0Coords[0]) * (180 / math.pi)

        self.logger.debug('Coxa Angle (theta1): %f' % theta1)

        #Middle joint angle in degrees.
        theta2 = ((IKA1 + IKA2) * (180 / math.pi))
        self.logger.debug('Femur Angle (theta2): %f' % theta2)

        #Outer joint angle in degrees.
        try:
            theta3 = math.acos((b ** 2 - self.link3.a ** 2 - self.link2.a ** 2) / (-2 * self.link3.a * self.link2.a)) * (180 / math.pi) - 180
            self.logger.debug('Tibia Angle (theta3): %f' % theta3)
        except:
            raise InvalidIKInput('Could not calculate tibia joint angle (theta3)!')

        #Set rotational angles (thetas) in links
        self.link1.setTheta(theta1)
        self.link2.setTheta(theta2)
        self.link3.setTheta(theta3)

