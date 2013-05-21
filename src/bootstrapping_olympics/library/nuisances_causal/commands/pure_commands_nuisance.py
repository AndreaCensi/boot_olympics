from .pure_commands import PureCommands
from blocks import SimpleBlackBox
from blocks.utils import WithQueue
from bootstrapping_olympics import (RepresentationNuisanceCausal, BootSpec,
    BootWithInternalLog)
from contracts import contract
import warnings

class PureCommandsNuisance(RepresentationNuisanceCausal):
    
    @contract(delta='float,>0', n='int,>=2')
    def __init__(self, delta, n):
        self.n = n
        self.delta = delta
        
    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        warnings.warn('Think this over')
        return spec    
    
    @contract(returns=SimpleBlackBox)
    def get_pre(self):

        class PureCommandsPreFilter(BootWithInternalLog, WithQueue):
   
            def __init__(self, n, interval):
                WithQueue.__init__(self)
                self.n = n
                self.interval = interval
                
            def __repr__(self):
                return 'PureCommandsPreFilter(%s,%s)' % (self.n, self.interval)
            
            @contract(x='tuple(float, *)')
            def put(self, x):
                t, cmd = x
                # multiply the commands
                for i in range(self.n):
                    t1 = t + i * self.interval
                    self.queue.append((t1, cmd))
                
        return PureCommandsPreFilter(n=self.n, interval=self.delta / self.n)
        
    @contract(returns=SimpleBlackBox)
    def get_post(self):
 
        class PostFilter(BootWithInternalLog, WithQueue):
            
            def __init__(self, delta):
                WithQueue.__init__(self)
                self.delta = delta
                self.pc = PureCommands(delta=delta, new_behavior=True)
                self.last_out = None
                self.log_add_child('pc', self.pc)
                
            def __repr__(self):
                return 'PureCommandsPostFilter(%s)' % (self.delta)
                
            @contract(value='tuple(float, tuple( (None|tuple(*,str)), RobotObservations))')
            def put(self, value):
                t, (cmds, obs) = value
                if cmds is None:
                    self.info('skipping if we do not have commands')
                    return
                
                cmd, source = cmds  # @UnusedVariable
                self.pc.update(t, cmd, obs)
                
                last = self.pc.last()
                if last is not None:
                    self.info('Finally last = %s' % str(last))
                    self.last_out = (t, obs)
                    self.queue.append(self.last_out)
                else:
                    self.info('No output generated from %s' % str(value))
             
                # send the first anyway
                if self.last_out is None:
                    self.last_out = (t, obs)
                    self.queue.append(self.last_out)
                    
        return PostFilter(self.delta)



        
    
