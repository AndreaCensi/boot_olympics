from bootstrapping_olympics import(BasicAgent, ExploringAgent,
    PredictingAgent, ServoingAgent, ServoAgentInterface, PredictorAgentInterface,
    LearningAgent)
from blocks.interface import Sink, SimpleBlackBox
from blocks.library import Instantaneous
from blocks.library.timed.checks import check_timed_named
from contracts import contract
from blocks.library.simple.with_queue import WithQueue


__all__ = [
    'RandomAgent',
    'RandomAgentPredictor',
    'RandomAgentServo',
]


class RandomAgent(BasicAgent, LearningAgent, PredictingAgent, 
                  ExploringAgent, ServoingAgent):
    ''' A dummy agent that gives random commands. '''

    def __init__(self):
        self.inited = False

    def init(self, boot_spec):
        self.boot_spec = boot_spec
        self.inited = True
        
    def get_explorer(self):
        
        class RandomAgentExplorer(Instantaneous):
            def __init__(self,boot_spec):
                self.boot_spec = boot_spec
            def transform_value(self, value):
                check_timed_named(value)
                (timestamp, (signal, _)) = value

                if not signal in ['observations']:
                    msg = 'Invalid signal %r to explorer.' % signal
                    raise ValueError(msg)

                cmd = self.boot_spec.get_commands().get_random_value()

                return timestamp, ('commands', cmd)
            
        return RandomAgentExplorer(self.boot_spec) 
            
        
    def get_learner_as_sink(self):
        
        class RandomAgentLearner(Sink):
            def reset(self):
                pass
            def put(self, value, block=True, timeout=None):  # @UnusedVariable
                check_timed_named(value)
                timestamp, (signal, obs) = value  # @UnusedVariable
                if not signal in ['observations', 'commands']:
                    msg = 'Invalid signal %r to learner.' % signal
                    raise ValueError(msg)
                
        return RandomAgentLearner()
    
    @contract(returns=SimpleBlackBox)
    def get_servo_system(self):
        
        class RandomAgentServoSystem(WithQueue):
            
            def __init__(self,boot_spec):
                self.boot_spec = boot_spec

            def reset(self):
                self.info('resetting servo, goal = None')
                WithQueue.reset(self)
                self.goal = None
        
            def put_noblock(self, value):
                check_timed_named(value)
                (timestamp, (signal, x)) = value

                self.info('received %s : %s' % (timestamp, signal))

                if not signal in ['observations', 'goal_observations']:
                    msg = 'Invalid signal %r to explorer.' % signal
                    raise ValueError(msg)
                
                if signal == 'goal_observations':
                    self.goal = x
                    self.info('received goal: %s' % self.goal)
                elif signal == 'observations':
                    if self.goal is not None:
                        cmd = self.boot_spec.get_commands().get_random_value()
                        self.append((timestamp, ('commands', cmd)))
                    else:
                        self.info('Still waiting for "goal_observations"')
                        pass # no commands

        return RandomAgentServoSystem(self.boot_spec)
                            
   
    def __repr__(self):
        return "RandomAgent"

    def get_servo(self):
        if not self.inited:
            raise Exception('get_servo() called before init().')
        return RandomAgentServo()

    def get_predictor(self):
        if not self.inited:
            raise Exception('get_predictor() called before init().')
        return RandomAgentPredictor()


class RandomAgentServo(ServoAgentInterface):

    def __init__(self):
        self.goal_called = False
        
    def init(self, boot_spec):
        self.boot_spec = boot_spec

    def set_goal_observations(self, goal):  # @UnusedVariable
        self.goal_called = True

    def process_observations(self, obs):
        pass

    def choose_commands(self):
        if not self.goal_called:
            msg = 'choose_commands() called before set_goal_observations()'
            raise Exception(msg)

        return self.boot_spec.get_commands().get_random_value()


class RandomAgentPredictor(PredictorAgentInterface):

    def __init__(self):
        pass
        
    def init(self, boot_spec):
        self.boot_spec = boot_spec

    def process_observations(self, obs):
        pass

    def predict_y(self, dt):  # @UnusedVariable
        return self.boot_spec.get_observations().get_random_value()

    def estimate_u(self):
        """ Estimate current u """
        return self.boot_spec.get_commands().get_random_value()


