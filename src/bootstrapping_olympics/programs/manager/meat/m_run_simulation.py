from . import contract, logger
from bootstrapping_olympics import (RobotInterface, RobotObservations,
    AgentInterface, ObsKeeper)
import time
import warnings


@contract(id_robot='str', id_agent='str',
          robot=RobotInterface, agent=AgentInterface, max_observations='>=1',
          max_time='>0')
def run_simulation(id_robot, robot, id_agent, agent, max_observations,
                   max_time,
                   check_valid_values=True, id_episode=None):
    ''' 
        Runs an episode of the simulation. The agent should already been
        init()ed. 
    '''
    episode = robot.new_episode()

    keeper = ObsKeeper(boot_spec=robot.get_spec(), id_robot=id_robot)

    if id_episode is None:
        id_episode = episode.id_episode

    id_world = episode.id_environment

    counter = 0
    obs_spec = robot.get_spec().get_observations()
    cmd_spec = robot.get_spec().get_commands()

    logger.info('Episode %s started (%s)' % (id_episode, episode))
    logger.info('max_observations: %s' % max_observations)
    logger.info('max_time: %s' % max_time)

    def get_observations():
        try:
            obs = robot.get_observations()
        except RobotObservations.NotReady:
            raise
        if check_valid_values:
            assert isinstance(obs, RobotObservations)
            obs_spec.check_valid_value(obs.observations)
            cmd_spec.check_valid_value(obs.commands)

        observations = keeper.push(timestamp=obs.timestamp,
                                   observations=obs.observations,
                                   commands=obs.commands,
                                   commands_source=obs.commands_source,
                                   id_episode=id_episode,
                                   id_world=id_world)

        if check_valid_values:
            obs_spec.check_valid_value(observations['observations'])
            cmd_spec.check_valid_value(observations['commands'])

        return observations, obs.episode_end

    
    while counter < max_observations:
        
        try:
            observations, episode_end = get_observations()
        except RobotObservations.NotReady:
            warnings.warn('remove')
            time.sleep(0.001)
            continue
        
        # print('counter %d ' % counter)

        yield observations

        if observations['time_from_episode_start'] > max_time:
            logger.info('Episode ended at %s for time limit %s > %s ' % 
                         (counter, observations['time_from_episode_start'],
                          max_time))
            break

        if episode_end:  # Fishy
            logger.info('Episode ended at %s due to obs.episode_end.'
                         % counter)
            break

        agent.process_observations(observations)
        commands = agent.choose_commands()  # repeated

        if check_valid_values:
            cmd_spec.check_valid_value(commands)

        robot.set_commands(commands, id_agent)

        counter += 1

