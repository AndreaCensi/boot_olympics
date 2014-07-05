from contracts import contract

from blocks import Identity, SimpleBlackBox, WithQueue
from bootstrapping_olympics import (BootSpec, RepresentationNuisanceCausal, StreamSpec)
from decent_logs import WithInternalLog
from streamels import check_streamels_1D, make_streamel_bit, streamels_join_1D


__all__ = ['ActiveNuisance']


class ActiveNuisance(RepresentationNuisanceCausal):
    
    """ 
        In this example, we add one command to the interface
        which takes values 0 or 1.
        
        When it is 0, nothing happens. When it is 1, it 
        the observations are inverted.
    """
            
    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        obs = spec.get_observations()
        cmd = spec.get_commands()
        # get the streamels
        streamels1 = cmd.get_streamels()
        check_streamels_1D(streamels1)
        # the extra command
        extra_cmd = make_streamel_bit()
        # join the two
        streamels2 = streamels_join_1D(streamels1, extra_cmd) 
        cmd2 = StreamSpec(id_stream=None, streamels=streamels2)
        spec = BootSpec(obs, cmd2)
        return spec    
    
    @contract(returns=SimpleBlackBox)
    def get_pre(self):        
        return Identity()
        
    @contract(returns=SimpleBlackBox)
    def get_post(self):
 
        class PostFilter(WithInternalLog, WithQueue):
            
            def __repr__(self):
                return 'PostFilter()'
                
            @contract(value='tuple(float, tuple( (None|tuple(*,str)), RobotObservations))')
            def put_noblock(self, value):
                t, ((cmds, source), obs) = value
                self.info('found cmds: %s' % str(cmds))
                bit = cmds[-1] > 0
                self.info('found bit: %r' % bit)
                if bit:
                    self.info('inverting')
                    obs.observations = 1 - obs.observations
                else:
                    obs.observations = obs.observations
                self.queue.append((t, obs))
                    
        return PostFilter()



        
    
