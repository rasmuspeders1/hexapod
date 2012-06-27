'''
Created on Aug 28, 2011

@author: rasmus
'''
import logging
import kinematics
import time
from math import *
import numpy
import sys

class GaitEngine:
    """
    Class used to generate gaits for a hexapod robot
    Also takes care of different stances/poses such as when in rest/sleep
    """
    def __init__(self, hexapod):
        self.logger = logging.getLogger('Gait Engine')
        self.logger.info('Gait Engine Initializing')

        self.hexapod = hexapod

        #time of last update time
        self.last_update = time.time()

        #helper unit vectors

        self.unit_row_x_vector = numpy.matrix([[1, 0, 0]])
        self.unit_row_y_vector = numpy.matrix([[0, 1, 0]])
        self.unit_row_z_vector = numpy.matrix([[0, 0, 1]])

        #type of gait to be generated
        self.gait_type = 'tripod'

        self.gait_cycle_methods = {'tripod': self.start_next_tripod_cycle}
        self.gait_update_methods = {'tripod': self.tripod_update}

        #Cycle counter
        self.cycle = 0

        #current cycle time.
        self.t_cycle = 0.0

        #time of last call to update method.
        self.last_update = 0.0

        #maximum positional update difference in millimetres
        self.max_pos_step_size = 2.0

        #=======================================================================
        # Body position and rotation variables
        #=======================================================================

        # Current position of body in global frame
        self.body_pos = numpy.matrix([[0.0], [0.0], [0.0]])

        # Desired position destination of body at the current time.
        # This is used if the desired body position can not be reached within 
        # one cycle.  
        self.body_dest = numpy.matrix([[0.0], [0.0], [0.0]])

        #current 
        self.body_rot = numpy.matrix([[0.0], [0.0], [0.0]])

        #free height under the robot
        self.body_pos_offset = numpy.matrix([[0.0], [0.0], [15.0]])

        #=======================================================================
        # Gait vars. these determine speed, step length and lift height (shape)
        #=======================================================================
        #average length of steps in mm
        self.__step_length = 50.0

        #maximal step lengt in mm
        self.max_step_length = 60.0

        #deprecated bezier based foot trajectory variables
        #TODO: remove when potential field is implemented
        self.foot_lift = 10.0
        self.bezier_delta1 = numpy.matrix([[0.0], [10.0], [30.0]])
        self.bezier_delta2 = numpy.matrix([[0.0], [-10.0], [30.0]])

        #desired speed of gait in mm/sec
        self.speed = 200.0

        # Gait_translation of movement relative to global frame
        # given as unit column vector
        self.gait_translation = None
        self.gait_rotation = None
        self.gait_max_rotation = numpy.matrix([[0.0], [0.0], [30.0]])
        self.gait_min_rotation = numpy.matrix([[0.0], [0.0], [-30.0]])
        self.gait_transform_matrix = None
        self.gait_transform_matrix_new = None

        #call set_gait_direction method to calculate initial translation vector
        self.set_gait_direction(0.0, 0.0)
        #Call set_gait_rotation method to calculate initial rotation vector
        self.set_gait_rotation(-0.0)

        #time to complete one cycle. 
        #A cycle is the time it takes to take one step.
        #This is also set when calling set_speed()
        self.total_cycle_time = float(self.__step_length) / float(self.speed)

        #lists of free feet and feet on the ground. I.E. feet in the air and feet on the ground
        #these are values relevant for the tripod gait.
        self.free_feet = ['lf', 'lb', 'rm']
        self.ground_feet = ['rf', 'rb', 'lm']

        #dictionary of current feet positions in global frame coordinates.
        #values here must be sane initial feet positions, relative to the body 
        #coordinates. whixh should be 0,0,0
        self.feet_pos = {'lf':numpy.matrix([[  85.0], [  75.0], [   0.0]]),
                         'lm':numpy.matrix([[   0.0], [  90.0], [   0.0]]),
                         'lb':numpy.matrix([[ -85.0], [  75.0], [   0.0]]),
                         'rf':numpy.matrix([[  85.0], [ -75.0], [   0.0]]),
                         'rm':numpy.matrix([[   0.0], [ -90.0], [   0.0]]),
                         'rb':numpy.matrix([[ -85.0], [ -75.0], [   0.0]])
                        }
        #destination of feet for current gait cycle
        self.feet_dest = {}

        #starting point of feet for current gait cycle
        self.feet_start = {}

        self.set_speed(80.0)

        #=======================================================================
        # Virtual Potential Field variables
        # Feet end points are by convention positively charged
        #=======================================================================

        #List of positions for positive charges
        self.positive_charges = []

        #list of positions for negative charges 
        self.negative_charges = []

    def get_feet_positions(self):
        """
        Method that returns feet positions. Call this method repeatedly to get 
        """
        return self.feet_pos;

    def get_body_position(self):
        """
        method returning the current body position
        """
        return self.body_pos

    def get_body_rotation(self):
        """
        Method that returns the current body rotation
        """
        return self.body_rot

    def update(self):
        """
        Method that updates the GaitEngine.
        Call this method with regular intervals to drive the GaitEngine.,
        This updates all positions according to the configuration.
        """
        #update cycle time. this indicates how long we are in the currents step cycle

        self.t_cycle += time.time() - self.last_update

        self.last_update = time.time()

        #if cycle is complete, start next cycle
        if self.t_cycle >= self.total_cycle_time:
            self.t_cycle = 0

            #before starting next cycle, make sure current cycle positions are exactly the destinations
            if self.cycle > 0:
                for foot in self.free_feet:
                    self.feet_pos[foot] = self.feet_dest[foot]

            self.gait_cycle_methods[self.gait_type]()

            self.cycle += 1

        self.gait_update_methods[self.gait_type]()
        #TODO: incorporate user controlled relative body movement.

    def tripod_update(self):
        """
        Update method for the tripod gait.
        This method calculates all incremental positions for the tripod gait.
        """
        t_norm = float(self.t_cycle) / float(self.total_cycle_time)

        #first calculate new foot placements for free feet
        for foot in self.free_feet:
            #If foot has already reached its destination do not update position for this foot.
            if (self.feet_start[foot] == self.feet_dest[foot]).all():
                continue

            self.feet_pos[foot] = self.bezier(self.feet_start[foot],
                                             self.feet_start[foot] + self.bezier_delta1,
                                             self.feet_dest[foot] + self.bezier_delta2,
                                             self.feet_dest[foot],
                                             self.t_cycle, self.total_cycle_time)

            #TODO: implement virtual potential field path planning for foot trajectories.

        #calculate and set body position and rotation from feet positions.
        self.__set_body_pos_rot_from_feet_pos()

    def __set_body_pos_rot_from_feet_pos(self):
        '''
        Method that calculates appropriate body position and rotation from the postion of the feet.
        '''
        #calculate body position relative to feet positions
        #TODO: fix body bobbing up and down.
        self.body_dest = numpy.mean(numpy.hstack([f for f in self.feet_pos.values()]) , 1)
        self.body_dest[2] = numpy.mean(numpy.hstack([self.feet_pos[p] for p in self.ground_feet]) , 1)[2]
        self.body_dest += self.body_pos_offset

        if numpy.linalg.norm(self.body_dest - self.body_pos) != 0.0:
            body_move_vec = self.body_dest - self.body_pos
            if numpy.linalg.norm(body_move_vec) > self.max_pos_step_size:
                body_move_vec = self.max_pos_step_size * body_move_vec / numpy.linalg.norm(body_move_vec)
            self.body_pos += body_move_vec

        #Calculate body rotation relative to feet positions
        #The yaw of the body must be in line with the polygon spanned by the feet positions.
        left_dir_vec = (self.feet_pos['lf'] - self.feet_pos['lb'])
        right_dir_vec = (self.feet_pos['rf'] - self.feet_pos['rb'])
        mean_dir_vec = numpy.mean(numpy.hstack([left_dir_vec, right_dir_vec]), 1)

        body_yaw = degrees(atan2(mean_dir_vec[1], mean_dir_vec[0]))

        self.body_rot[2] = body_yaw

        #body_yaw = 

    def get_spline_via(self, start, destination):
        '''
        Method that calculates an appropriate via point for the foot spline 
        trajectory.
        '''
        #Get position halfway between start and destination
        p_half = start + (destination - start) * 0.5
        via = p_half + numpy.matrix([[0.0], [0.0], [self.foot_lift]])
        self.logger.info('Start:\n%s\nDest:\n%s\nVia:\n%s', start, destination, via)

        return via

    def spline(self, p0, p1, p2, p3, t, t_total):
        '''
        Method that calculates a position on a curve interpolated between 4 points.
        Takes numpy matrices as input.
        '''
        t_norm = (t / t_total)

        pos = 0.5 * ((2 * p1) + (-p0 + p2) * t_norm + (2 * p0 - 5 * p1 + 4 * p2 - p3) * (t_norm ** 2) + (-p0 + 3 * p1 - 3 * p2 + p3) * (t_norm ** 3))

        return pos

    def start_next_tripod_cycle(self):
        """
        cycle method for tripod gait
        """
        #self.logger.info('Cycle: %d', self.cycle)
        self.ground_feet, self.free_feet = self.free_feet, self.ground_feet

        if (self.gait_transform_matrix != self.gait_transform_matrix_new).any() and self.cycle % 2 == 0:
            self.gait_transform_matrix = self.gait_transform_matrix_new

        cycle_gait_t_matrix = self.gait_transform_matrix
        #self.logger.info('Gait T Matrix:\n%s\nNew Gait T Matrix:\n%s', self.gait_transform_matrix, self.gait_transform_matrix_new)

        #self.logger.info('Gait T Matrix:\n%s', cycle_gait_t_matrix)

        if self.cycle == 0:
            for foot in self.free_feet:
                self.feet_start[foot] = self.feet_pos[foot]
                self.feet_dest[foot] = (self.hexapod.body.get_t_matrix() * kinematics.get_t_matrix(self.gait_translation / 2, self.gait_rotation / 2) * self.hexapod.body.get_i_t_matrix() * numpy.append(self.feet_start[foot], numpy.matrix([1]), axis=0))[0:3]
        else:
            for foot in self.free_feet:
                self.feet_start[foot] = self.feet_pos[foot]
                self.feet_dest[foot] = (cycle_gait_t_matrix * numpy.append(self.feet_start[foot], numpy.matrix([1]), axis=0))[0:3]

    def set_speed(self, speed):
        """
        Method that sets the gait speed in mm/sec 
        """
        try:
            self.speed = float(speed)
        except:
            raise TypeError('Input must be float or be valid argument for float()')

        self.__step_length = min(self.speed / 2.0, self.max_step_length)

        self.foot_lift = max(10.0, min(self.__step_length / 10.0, 30.0))

        self.bezier_delta1 = numpy.matrix([[0.0], [0.0], [self.foot_lift]])
        self.bezier_delta2 = numpy.matrix([[0.0], [0.0], [self.foot_lift]])

        if self.speed != 0:
            self.total_cycle_time = self.__step_length / self.speed

    def set_translation(self, x, y):
        '''
        Method that sets the overall gait translation
        input is a directional vector with speed determined by its length,
        which must be between 0 and 1
        '''
        self.set_gait_direction(x, y)
        self.set_speed((x ** 2 + y ** 2) ** 0.5 * 100.0)

    def set_gait_direction(self, x, y):
        """
        Method that sets the movement gait_translation of the gait.
        input must be given as unit gait_translation column vector in the global frame
        """

        #convert input tuple to numpy matrix
        self.gait_translation = numpy.matrix([[float(x)],
                                       [float(y)],
                                       [0]
                                       ])

        #make sure gait_translation vector is normalized
        if not numpy.linalg.norm(self.gait_translation) == 0:
            self.gait_translation = (self.gait_translation / numpy.linalg.norm(self.gait_translation)) * self.__step_length

        self.update_gait()

    def set_gait_rotation(self, yaw):
        """
        Method that sets the gait_rotation of the gait.
        input must be given as angles of rotation in degrees
        """
        #convert input to rotation angle tuple. (roll, pitch, yaw)
        self.gait_rotation = numpy.matrix([[0.0], [0.0], [float(yaw)]])
        for i, v in enumerate(self.gait_rotation):
            if v > self.gait_max_rotation[i]:
                self.logger.warning('Gait rotation on axis %d reached maximum of %d degrees.', i, self.gait_max_rotation[i])
                self.gait_rotation[i] = self.gait_max_rotation[i]

            if v < self.gait_min_rotation[i]:
                self.logger.warning('Gait rotation on axis %d reached minimum of %d degrees.', i, self.gait_min_rotation[i])
                self.gait_rotation[i] = self.gait_min_rotation[i]

        self.update_gait()

    def update_gait(self):
        '''
        Method that updates the gait with input from gait translation and rotation vectors.
        Updates gait transformation matrix and places virtual potential field charges. 
        '''
        if self.gait_translation is None and self.gait_rotation is None:
            self.logger.error('Cannot update Gait Transformation matrix when both translation and rotations vectors are None.')
            return


        if self.gait_translation is None:
            self.gait_transform_matrix_new = self.hexapod.body.get_t_matrix() * kinematics.get_t_matrix(numpy.matrix([[0.0], [0.0], [0.0], [1.0]]), self.gait_rotation) * self.hexapod.body.get_i_t_matrix()
        elif self.gait_rotation is None:
            self.gait_transform_matrix_new = self.hexapod.body.get_t_matrix() * kinematics.get_t_matrix(self.gait_translation, numpy.matrix([[0.0], [0.0], [0.0]])) * self.hexapod.body.get_i_t_matrix()
        else:
            self.gait_transform_matrix_new = self.hexapod.body.get_t_matrix() * kinematics.get_t_matrix(self.gait_translation, self.gait_rotation) * self.hexapod.body.get_i_t_matrix()
        if self.gait_transform_matrix is None:
            self.gait_transform_matrix = self.gait_transform_matrix_new

        # TODO: Place virtual potential field charges


    def bezier(self, p0, p1, p2, p3, t, t_total):
        t_norm = float(t) / float(t_total)
        return (1 - t_norm) ** 3 * p0 + 3 * (1 - t_norm) ** 2 * t_norm * p1 + 3 * (1 - t_norm) * t_norm ** 2 * p2 + t_norm ** 3 * p3

    def incenter(self, p0, p1, p2):
        """
        method that returns the coordinates of the incenter of the triangle with vertices in p0, p1 and p2
        """
        #a is the length of the side opposite p0
        a = numpy.linalg.norm(p1 - p2)
        #b is the length of the side opposite p1
        b = numpy.linalg.norm(p2 - p0)
        #c is the length of the side opposite p2
        c = numpy.linalg.norm(p0 - p1)

        sides = numpy.matrix([[numpy.linalg.norm(p1 - p2)], [numpy.linalg.norm(p2 - p0)], [numpy.linalg.norm(p0 - p1)]])

        triangle = numpy.append(p0, p1, axis=1)
        triangle = numpy.append(triangle, p2, axis=1)

        incenter = (triangle * sides) / (a + b + c)

        return incenter



