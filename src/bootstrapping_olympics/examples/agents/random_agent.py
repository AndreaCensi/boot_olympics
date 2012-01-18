from ...interfaces import AgentInterface

__all__ = ['RandomAgent']


class RandomAgent(AgentInterface):

    def init(self, boot_spec):
        self.boot_spec = boot_spec

    def process_observations(self, observations):
        pass

    def choose_commands(self):
        return self.boot_spec.get_commands().get_random_value()

    def __repr__(self):
        return "RandomAgent"

    def get_servo(self):
        return RandomAgentServo(self.boot_spec)

    def get_predictor(self):
        return RandomAgentPredictor(self.boot_spec)


class RandomAgentServo():

    def __init__(self, boot_spec):
        self.boot_spec = boot_spec

    def set_goal_observations(self, goal):
        pass

    def process_observations(self, obs):
        pass

    def choose_commands(self):
        return self.boot_spec.get_commands().get_random_value()


class RandomAgentPredictor():

    def __init__(self, boot_spec):
        self.boot_spec = boot_spec

    def process_observations(self, obs):
        pass

    def predict_y(self, dt):
        return self.boot_spec.get_observations().get_random_value()

