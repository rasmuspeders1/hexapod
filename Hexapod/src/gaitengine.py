'''
Created on Aug 28, 2011

@author: rasmus
'''
import logging
import kinematics
import time
from math import *
import numpy

class GaitEngine:
    """
    Class used to generate gaits for a hexapod robot
    Also takes care of different stances/poses such as when in rest/sleep 
    """
    def __init__(self):
        self.logger = logging.getLogger('Gait Engine')
        self.logger.info('Gait Engine Initializing')
        
        #time of last update time
        self.last_update = time.time()
        
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
        
        #=======================================================================
        # Relative body movement vars. 
        # body_pos. body_start and body_dest are used for moving the 
        # body during the gait.
        # body speed determines the speed of the directly user controlled body
        # motion.
        #=======================================================================
        #self.body_speed = 0
        
        #self.body_direction = numpy.matrix([[0], [0], [0]])
        
        
        #current position of body in global frame
        self.body_pos = numpy.matrix([[0], [0], [10]])
        #destination of body in current gait cycle
        self.body_dest = numpy.matrix([[0], [0], [0]])
        #starting point for body in current gait cycle
        self.body_start = numpy.matrix([[0], [0], [0]])
        
        self.body_rot = numpy.matrix([[0], [0], [0]])
        self.body_rot_start = numpy.matrix([[0], [0], [0]])
        self.body_rot_dest = numpy.matrix([[0], [0], [0]])
        
        #=======================================================================
        # Gait vars. these determine speed, step length and lift height (shape)
        #=======================================================================
        #average length of steps in mm
        self.step_length = 50.0
        self.max_step_length = 80.0
        self.feet_lift = 10.0
        self.bezier_delta1 = numpy.matrix([[0], [0], [30]])        
        self.bezier_delta2 = numpy.matrix([[0], [0], [30]])
        #speed of gait in mm/sec
        self.speed = 50.0
        
        
        #=======================================================================
        # Translate only flag. 
        # When this is true the robot will only translate in the gait direction.
        # When this is false the robot will turn to head in the the gait 
        # direction.
        #=======================================================================
        self.translate_only = False
        
        #gait_direction of movement relative to global frame
        #Given as unit column vector
        self.gait_direction = None

        #call set_gait_direction method to calculate initial move_vector
        self.set_gait_direction((1, 0, 0))
        
        #time to complete one cycle. 
        #A cycle is the time it takes to take one step.
        #This is also set when calling set_speed()
        self.total_cycle_time = float(self.step_length) / float(self.speed)
        
        #lists of free feet and feet on the ground. I.E. feet in the air and feet on the ground
        #these are values relevant for the tripod gait.
        self.free_feet = ['lf', 'lb', 'rm']
        self.ground_feet = ['rf', 'rb', 'lm']
        
        #dictionary of current feet positions in global frame coordinates.
        #values here must be sane initial feet positions, relative to the body 
        #coordinates. whixh should be 0,0,0
        self.feet_pos = {'lf':numpy.matrix([[  85], [  90], [   0]]),
                         'lm':numpy.matrix([[   0], [  90], [   0]]),
                         'lb':numpy.matrix([[ -85], [  75], [   0]]),
                         'rf':numpy.matrix([[  85], [ -75], [   0]]),
                         'rm':numpy.matrix([[   0], [ -90], [   0]]),
                         'rb':numpy.matrix([[ -85], [ -75], [   0]])
                        }
        #destination of feet for current gait cycle
        self.feet_dest = {}
        
        #starting point of feet for current gait cycle
        self.feet_start = {}
        
        self.set_speed(150.0)

        
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
                    self.body_pos = self.body_dest
                    self.body_rot = self.body_rot_dest
            
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
            self.feet_pos[foot] = self.bezier(self.feet_start[foot],
                                              self.feet_start[foot] + self.bezier_delta1,
                                              self.feet_dest[foot] + self.bezier_delta2,
                                              self.feet_dest[foot],
                                              self.t_cycle, self.total_cycle_time)
        #calculate new body position
        self.body_pos = self.body_start + (self.body_dest - self.body_start) * t_norm
        #calculate body rotation
        self.body_rot = self.body_rot_start + (self.body_rot_dest - self.body_rot_start) * t_norm
            
    def start_next_tripod_cycle(self):
        """
        cycle method for tripod gait
        """
        
        self.ground_feet, self.free_feet = self.free_feet, self.ground_feet
        
        #calculate gait yaw (how much the next step should rotate the robot).
        gait_yaw = degrees(acos(numpy.dot(self.gait_direction.transpose(), numpy.matrix([[1.0], [0], [0]]))))
        self.logger.info('Gait Yaw: %f',gait_yaw)
        
        translate_vector = self.gait_direction * self.step_length
        
        t_matrix = kinematics.getTMatrix(translate_vector, (0, 0, gait_yaw))
        self.logger.info("Transformation Matrix:\n%s",str(t_matrix))
  
        
        if self.cycle == 0:
            for foot in self.free_feet:
                self.feet_start[foot] = self.feet_pos[foot]
                self.feet_dest[foot] = self.feet_start[foot] + self.gait_direction * (self.step_length / 2.0)
            
            #set relative body position start and destination
            self.body_start = self.body_pos
            self.body_dest = self.body_start + self.gait_direction * (self.step_length / 4.0)
                
        else:
            #set start and destination positions for free feet
            #TODO: implement turning gait.
            for foot in self.free_feet:
                self.feet_start[foot] = self.feet_pos[foot]
                
                self.feet_dest[foot] = self.feet_start[foot] + translate_vector
                #self.feet_dest[foot] = (t_matrix * numpy.append(self.feet_start[foot], numpy.matrix([0]), axis=0))[0:3]
                self.logger.info("\n%s\n%s\n%s", foot, str(t_matrix * numpy.append(self.feet_start[foot], numpy.matrix([0]), axis=0)), str(self.feet_dest[foot]) )
                
                
            #set relative body position start and destination
            self.body_start = self.body_pos            
            self.body_dest = self.body_start + self.gait_direction * (self.step_length / 2.0)
            
            #self.body_rot_start = self.body_rot
            #self.body_rot_dest = self.body_rot_start + numpy.matrix([[0], [0], [gait_yaw / 2.0]])
            
            
    
    def set_speed(self, speed):
        """
        Method that sets the gait speed in mm/sec 
        """
        try:
            self.speed = float(speed)
        except:
            raise TypeError('Input must be float or be valid argument for float()')
        
        self.step_length = min(self.speed / 2.0, self.max_step_length)
        
        self.feet_lift = max(10.0, min(self.step_length / 10.0, 30.0))
        
        self.bezier_delta1 = numpy.matrix([[0], [0], [self.feet_lift]])
        self.bezier_delta2 = numpy.matrix([[0], [0], [self.feet_lift]])  
        
        
        self.total_cycle_time = self.step_length / self.speed 

        
    def set_gait_direction(self, direction):
        """
        Method that sets the movement gait_direction of the gait.
        input must be given as unit gait_direction column vector in the global frame
        """
        if not type(direction) is tuple and len(direction) == 3:
            raise TypeError('Input Must be tuple of length 3.')
        
        #convert input tuple to numpy matrix
        self.gait_direction = numpy.matrix([[float(direction[0])],
                                       [float(direction[1])],
                                       [float(direction[2])]
                                       ])
        
        #make sure gait_direction vector is normalized
        self.gait_direction = self.gait_direction / numpy.linalg.norm(self.gait_direction)
    
    def set_body_direction(self, direction):
        """
        Method to set the relative movement gait_direction of the body.
        """
        if not type(direction) is tuple and len(direction) == 3:
            raise TypeError('Input Must be tuple of length 3.')
        
        #convert input tuple to numpy matrix
        self.body_direction = numpy.matrix([[direction[0]],
                                            [direction[1]],
                                            [direction[2]]
                                           ])
        
        #make sure gait_direction vector is normalized
        self.body_direction = self.body_direction / numpy.linalg.norm(self.body_direction)
        
        #calculate movement vector for relative body motion
        self.body_move_vector = self.body_direction * self.body_speed

    def set_body_speed(self, speed):
        """
        Method that sets the gait speed in mm/sec 
        """
        try:
            self.body_speed = float(speed)
        except:
            raise TypeError('Input must be float or be valid argument for float()')
        
        self.body_move_vector = self.body_direction * self.body_speed   
    
    def bezier(self, p0, p1, p2, p3, t, t_total):
        t_norm = float(t) / float(t_total)
        return (1 - t_norm) ** 3 * p0 + 3 * (1 - t_norm) ** 2 * t_norm * p1 + 3 * (1 - t_norm) * t_norm ** 2 * p2 + t_norm ** 3 * p3
    
