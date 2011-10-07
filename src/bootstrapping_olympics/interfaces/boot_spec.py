from . import StreamSpec, logger
from contracts import check, describe_type, contract
from pprint import pformat
import numpy as np

class BootSpec:
    ''' 
        This class represents the interface of a sensorimotor
        interface. For now it specifies the shape of the sensel
        array and the number of commands. 
    '''
    
    def __init__(self, obs_spec, cmd_spec, id_robot=None, desc=None, extra=None):
        self.id_robot = id_robot
        self.desc = desc
        self.extra = extra
        if not isinstance(obs_spec, StreamSpec):
            msg = ('Expected StreamSpec for observations, not %s.' % 
                   describe_type(obs_spec))
            raise ValueError(msg)
        if not isinstance(cmd_spec, StreamSpec):
            msg = ('Expected StreamSpec for commands, not %s.' % 
                   describe_type(cmd_spec))
            raise ValueError(msg)
        self.observations = obs_spec
        self.commands = cmd_spec

    @contract(returns=StreamSpec)
    def get_commands(self):
        ''' Returns a StreamSpec instance representing the commands. '''
        return self.commands
    
    @contract(returns=StreamSpec)
    def get_observations(self):
        ''' Returns a StreamSpec instance representing the observations. '''
        return self.observations
    
    def __eq__(self, other):
        return ((self.commands == other.commands) and 
                (self.observations == other.observations)) 
     
    def reshape_raw_sensels(self, raw): # XXX: remove
        ''' Transforms the raw 1D sensels in ROS messages to
            a correctly shaped array.
        '''
        return np.array(raw, dtype='float32').reshape(self.sensels_shape)
        
    def check_compatible_raw_sensels_values(self, sensels):
        if len(sensels) != self.num_sensels:
            msg = ('Raw sensels incompatible with spec %s: expected len %s, got %s.' % 
                   (self, self.num_sensels, len(sensels)))
            raise Exception(msg)
        
    def check_compatible_sensels_values(self, sensels):
        ''' Returns true if the given sensels (numpy array) 
            is compatible with this sensorimotor cascade. '''
        # XXX: this is wasteful
        sensels = np.array(sensels)
        ok_shape = self.sensels_shape == sensels.shape
        if not ok_shape:
            msg = ('Sensels incompatible with spec %s: expected shape %s, got %s.' % 
                   (self, self.sensels_shape, sensels.shape))
            raise Exception(msg)
        
    def check_compatible_commands_values(self, commands0):
        commands = np.array(commands0)
        ok_shape = commands.size == self.num_commands
        if not ok_shape:
            msg = ('Commands incompatible with spec %s: expected %d commands, got %d. %s' % 
                   (self, self.num_commands, commands.size, self.commands_spec))
            raise Exception(msg)
        
        for i in range(self.num_commands):
            u_i = commands[i]
            bounds = self.commands_spec[i]
            if isinstance(bounds, tuple):
                umin, umax = bounds
                if not umin <= u_i <= umax:
                    msg = ('Invalid value u[%d]=%d not in [%s,%s] in u=%s' % 
                           (i, u_i, umin, umax, commands0))
                    raise Exception(msg)
            else:
                # TODO: not implemented yet
                pass
    
    @staticmethod
    def check_valid_commands_spec(commands_spec):
        # XXX
        check('list', commands_spec)

    @staticmethod
    def from_ros_structure(msg):
        ''' Infers the boot spec from a ROS message. '''
        commands_spec = eval(msg.commands_spec) # XXX: not safe 
        sensels_shape = msg.sensel_shape # TODO: check coherence
        return BootSpec(sensels_shape, commands_spec)
    
    def __str__(self):
        return 'BootSpec(%s,%s)' % (self.observations, self.commands)

    @staticmethod
    def from_yaml(xo):
        try:
            if not isinstance(xo, dict):
                raise ValueError('Expected a dict, got %s' % xo)
            x = dict(**xo) # make a copy
            observations = StreamSpec.from_yaml(x.pop('observations'))
            commands = StreamSpec.from_yaml(x.pop('commands'))
            extra = x.pop('extra', None)
            desc = x.pop('desc', None)
            id_robot = x.pop('id', None)
            
            if x.keys():
                logger.warning('While reading\n%s\nextra keys detected: %s' % 
                               (pformat(xo), x.keys()))
            return BootSpec(observations, commands,
                            id_robot=id_robot,
                            extra=extra,
                            desc=desc)
        except:
            logger.error('Error while parsing the BootSpec:\n%s' % xo)
            raise
        
    def to_yaml(self):
        x = {}
        x['observations'] = self.observations.to_yaml()
        x['commands'] = self.commands.to_yaml()
        x['extra'] = self.extra
        x['id'] = self.id_robot
        x['desc'] = self.desc
        return x
    
