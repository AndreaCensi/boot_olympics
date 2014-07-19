from ..meat import simulate_agent_robot
from .main import BOM
from bootstrapping_olympics.utils import unique_timestamp_string


class CmdSimulate(BOM.get_sub()):
    '''Simulate the interaction of an agent and a robot. '''

    cmd = 'simulate'

    def define_program_options(self, params):
        params.add_string('agent', short='-a')
        params.add_string('robot', short='-r')

        params.add_flag("stateful",
                      help="Save/load the state of the agent.")
        params.add_int("num_episodes", default=10,
                          help="Number of episodes to simulate ")
        params.add_flag("cumulative", help="Count already simulated episodes")
        params.add_flag("extra", help="Writes extra information")
        params.add_float("episode_len", default=30,
                          help="Maximum len of episode (seconds)")


    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()


        id_agent = options.agent
        id_robot = options.robot
        
        id_episodes = [unique_timestamp_string()+'-%s' % i 
                       for i in range(options.num_episodes)] 
        simulate_agent_robot(data_central,
                 id_agent=id_agent,
                 id_robot=id_robot,
                 max_episode_len=options.episode_len,
                 stateful=options.stateful,
                 cumulative=options.cumulative,
                 write_extra=options.extra,
                 id_episodes=id_episodes)
