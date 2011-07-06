# I have no idea

class Simulation:

    def current_time(self):
        ''' Returns the current time. '''
        
    def current_observations(self):
        pass
    def random_robot_state(self):
        ''' Obtains a random robot state. '''
    def current_robot_state(self):
        pass
    def set_robot_state(self):
        pass
    def random_commands(self):
        ''' Returns a vector of random commands. '''
        pass

    # Run the system
    def execute_motion(self, commands, interval):
        ''' Run the system with the given commands for the given interval. '''
        pass

    def execute_closed_loop(self, interval):
        ''' Runs the system closed-loop for a given interval. '''
        pass
        
    def execute_closed_loop_until_stops(self, timeout=None):
        ''' Runs the system in closed-loop until the agent decides to stop. '''
        pass
    
class Task:
    def __init__(self, requires_simulation=False):
        pass
    
class ServoTaskInstant(Task):
    
    def __init__(self, delta):
        self.delta = delta
        self.task = 'servo_instant'
        Task.__init__(self, requires_simulation=True)
    
    def go(self, sim):
        # Reset the robot
        sim.new_episode()
        start_pose = sim.random_state()
        sim.set_robot_pose(start_pose)
        start_observations = sim.current_observations()
        cmds = sim.random_commands()
        sim.execute_motion(commands=cmds, commands=self.delta)
        final_pose = sim.current_robot_pose()
        final_observations = self.current_observations()

        query = dict(observations=start_observations,
                     goal=final_observations)
        response = self.query_agent(task=self.task, query, needs=['commands'])
        chosen = response['commands']
        result = {
            'start_pose': start_pose,
            'final_pose': final_pose,
            'start_observations': start_observations,
            'final_observations': final_observations,
            'commands': cmds,
            'chosen': chosen,
        }
        return result

class SimpleMetric:
    def compute(self, results):
        for r in results:
            pass
    
