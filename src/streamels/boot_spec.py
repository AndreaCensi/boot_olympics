from pprint import pformat

from contracts import  new_contract, contract
from contracts.utils import check_isinstance

from streamels import XStreamSpec, logger


__all__ = ['BootSpec']


class BootSpec(object):
    ''' 
        This class describes input/output of a sensorimotor cascade.
    '''

    @contract(obs_spec=XStreamSpec, cmd_spec=XStreamSpec)
    def __init__(self, obs_spec, cmd_spec, id_robot=None,
                 desc=None, extra=None):
        self.id_robot = id_robot
        self.desc = desc
        self.extra = extra
        check_isinstance(obs_spec, XStreamSpec)
        check_isinstance(cmd_spec, XStreamSpec)
        self._observations = obs_spec
        self._commands = cmd_spec

    def __repr__(self):
        return 'BootSpec(%r,%r)' % (self._observations, self._commands)
    
    @contract(returns=XStreamSpec)
    def get_commands(self):
        ''' Returns a StreamSpec instance representing the commands. '''
        return self._commands

    @contract(returns=XStreamSpec)
    def get_observations(self):
        ''' Returns a StreamSpec instance representing the observations. '''
        return self._observations

    def __eq__(self, other):
        return ((self._commands == other._commands) and
                (self._observations == other._observations))

    def __str__(self):
        return 'BootSpec(%s,%s)' % (self._observations, self._commands)

    @staticmethod
    def from_yaml(xo):
        try:
            if not isinstance(xo, dict):
                raise ValueError('Expected a dict, got %s' % xo)
            x = dict(**xo)  # make a copy


            def check_contained(key, D, name='key'):
                ''' Raises a ValueError if key is not in the dictionary-like object D. '''
                if not key in D:
                    msg = 'No %s %r found among %s.' % (name, key, D.keys())
                    # TODO: pretty print
                    # TODO: only print a limited number
                    # TODO: add suggestions
                    raise ValueError(msg)

            check_contained('observations', x)
            check_contained('commands', x)
            observations = XStreamSpec.from_yaml(x.pop('observations'))
            commands = XStreamSpec.from_yaml(x.pop('commands'))
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
        x['observations'] = self._observations.to_yaml()
        x['commands'] = self._commands.to_yaml()
        x['extra'] = self.extra
        x['id'] = self.id_robot
        x['desc'] = self.desc
        return x

new_contract('BootSpec', BootSpec)

