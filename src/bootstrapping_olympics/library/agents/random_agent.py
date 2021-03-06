from bootstrapping_olympics import (PredictorAgentInterface, ServoAgentInterface,
    AgentInterface)

__all__ = ['RandomAgent', 'RandomAgentPredictor', 'RandomAgentServo']


class RandomAgent(AgentInterface):
    ''' A dummy agent that gives random commands. '''

    def __init__(self):
        self.inited = False
        self.process_called = True

    def init(self, boot_spec):
        self.boot_spec = boot_spec

        # Just some checks, useful for debugging
        if self.inited:
            raise Exception('Already init()ed once.')
        self.inited = True

    def process_observations(self, observations):  # @UnusedVariable
        if not self.inited:
            raise Exception('process_observations() called before init().')
        self.process_called = True

    def choose_commands(self):
        if not self.process_called:
            msg = 'choose_commands() called before process_observations().'
            raise Exception(msg)

        commands = self.boot_spec.get_commands().get_random_value()
        # self.info('commands: %r' % commands)
        return commands

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


