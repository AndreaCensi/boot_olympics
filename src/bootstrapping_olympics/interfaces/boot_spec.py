from . import StreamSpec, logger, contract, np
from contracts import  describe_type
from pprint import pformat


__all__ = ['BootSpec']

class BootSpec:
    ''' 
        This class represents the interface of a sensorimotor
        interface. 
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
    
