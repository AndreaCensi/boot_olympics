import numpy as np
from contracts.main import check

class BootSpec:
    ''' 
        This class represents the interface of a sensorimotor
        interface. For now it specifies the shape of the sensel
        array and the number of commands. 
    '''
    
    def __init__(self, sensels_shape, commands_spec):
        self.sensels_shape = sensels_shape
        self.commands_spec = commands_spec
        BootSpec.check_valid_commands_spec(commands_spec)
        self.num_sensels = np.prod(sensels_shape)
        self.num_commands = len(self.commands_spec)
    
    def __eq__(self, other):
        return ((self.sensels_shape == other.sensels_shape) and 
                (self.commands_spec == other.commands_spec))
        
    def check_compatible_sensels_values(self, sensels):
        ''' Returns true if the given sensels (numpy array) 
            is compatible with this sensorimotor cascade. '''
        sensels = np.array(sensels)
        ok_shape = self.sensels_shape == sensels.shape
        if not ok_shape:
            msg = ('Sensels incompatible with spec %s: expected shape %s, got %s.' % 
                   (self, self.sensels_shape, sensels.shape))
            raise Exception(msg)
        
    def check_compatible_commands_values(self, commands):
        commands = np.array(commands)
        ok_shape = commands.size == self.num_commands
        if not ok_shape:
            msg = ('Commands incompatible with spec %s: expected %d commands, got %d. %s' % 
                   (self, self.num_commands, commands.size, self.commands_spec))
            raise Exception(msg)
        
    
    @staticmethod
    def check_valid_commands_spec(commands_spec):
        check('list', commands_spec)

    @staticmethod
    def from_ros_structure(msg):
        ''' Infers the boot spec from a ROS message. '''
        commands_spec = eval(msg.commands_spec) # XXX: not safe 
        sensels_shape = msg.sensel_shape # TODO: check coherence
        return BootSpec(sensels_shape, commands_spec)
    
    def __str__(self):
        return 'Spec(%r,%r)' % (self.sensels_shape, self.commands_spec)