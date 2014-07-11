from ..meat import task_servo
from .main import BOM



class CmdServo(BOM.get_sub()):
    ''' Runs a servo experiment. '''

    cmd = 'servo'

    def define_program_options(self, params):
        params.add_string('agent', short='-a')
        params.add_string('robot', short='-r')

        params.add_flag("stateful",
                      help="Save/load the state of the agent.")
        params.add_int("num_episodes", default=10,
                          help="Number of episodes to simulate ")
        params.add_flag("cumulative", help="Count already simulated episodes")
        params.add_flag("extra", help="Writes extra information")
        params.add_float("interval_print", default=5,
                          help='Frequency of debug messages.')

        params.add_float("displacement", default=1.0, 
                          help="Initial robot displacement (seconds).")
        params.add_float("max_episode_len", default=30,
                          help="Maximum len of episode (seconds)")
    
    

    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()


        id_agent = options.agent
        id_robot = options.robot
        task_servo(data_central=data_central,
                 id_agent=id_agent,
                 id_robot=id_robot,
                 displacement=options.displacement,
                 max_episode_len=options.max_episode_len,
                 num_episodes=options.num_episodes,
                 cumulative=options.cumulative,
                 interval_print=options.interval_print,
                 num_episodes_with_robot_state=options.num_episodes) # xXX
    
