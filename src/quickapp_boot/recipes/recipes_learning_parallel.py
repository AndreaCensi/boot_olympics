from boot_manager import DataCentral
from contracts import contract
from quickapp import CompmakeContext, ResourceManager
from quickapp_boot import RM_AGENT_LEARN
from quickapp_boot.jobs import (jobs_parallel_learning,
    jobs_parallel_learning_concurrent, jobs_parallel_learning_concurrent_reps)


__all__ = ['recipe_agentlearn_by_parallel',
           'recipe_agentlearn_by_parallel_concurrent',
           'recipe_agentlearn_by_parallel_concurrent_reps']

@contract(context=CompmakeContext, data_central=DataCentral, episodes='list(str)',
          only_agents='None|list(str)')
def recipe_agentlearn_by_parallel(context, data_central, episodes,
                                  only_agents=None,
                                  only_robots=None,
                                  intermediate_reports=False,
                                  episodes_per_tranche=50):
    """
        provides:  agent-learn (id_agent, id_robot)
        
        learns all episodes for the robot in the db
    """

    rm = context.get_resource_manager()
    
    def rp_learn(context, id_agent, id_robot):
        if only_agents is not None:
            if not id_agent in only_agents:
                msg = 'Agent %r not in %r' % (id_agent, only_agents)
                raise ResourceManager.CannotProvide(msg)
            
        if only_robots is not None:
            if not id_robot in only_robots:
                msg = 'Robot %r not in %r' % (id_robot, only_robots)
                raise ResourceManager.CannotProvide(msg)
        
        return jobs_parallel_learning(context, data_central,
                                      id_agent, id_robot, episodes,
                                      intermediate_reports=intermediate_reports,
                                      episodes_per_tranche=episodes_per_tranche)
        
    rm.set_resource_provider(RM_AGENT_LEARN, rp_learn)


@contract(context=CompmakeContext, data_central=DataCentral, episodes='list(str)',
          only_agents='None|list(str)')
def recipe_agentlearn_by_parallel_concurrent(context, data_central, episodes, n=None,
                                             only_agents=None):
    """
        provides: agent-learn (id_agent, id_robot)
        
        learns all episodes for the robot in the db
        
        if only_agents is not None, then it is a list 
        of agents id that can use this recipe. 
        
    """
    import multiprocessing
    if n is None:
        n = multiprocessing.cpu_count()

    rm = context.get_resource_manager()
    
    def rp_learn(context, id_agent, id_robot):
        if only_agents is not None:
            if not id_agent in only_agents:
                msg = 'Agent %r not in %r' % (id_agent, only_agents)
                raise ResourceManager.CannotProvide(msg)
        return jobs_parallel_learning_concurrent(context, data_central,
                                      id_agent, id_robot, episodes, n=n)
        
    rm.set_resource_provider(RM_AGENT_LEARN, rp_learn)


@contract(context=CompmakeContext, data_central=DataCentral, episodes='list(str)',
          only_agents='None|list(str)', max_reps='int,>=2', n='None|int,>=2')
def recipe_agentlearn_by_parallel_concurrent_reps(context, data_central, episodes,
                                                  max_reps, n=None,
                                                  only_agents=None):
    """
        provides: agent-learn (id_agent, id_robot)
        
        learns all episodes for the robot in the db
        
        if only_agents is not None, then it is a list 
        of agents id that can use this recipe. 
        
    """
    import multiprocessing
    if n is None:
        n = multiprocessing.cpu_count()

    rm = context.get_resource_manager()
    
    def rp_learn(context, id_agent, id_robot):
        if only_agents is not None:
            if not id_agent in only_agents:
                msg = 'Agent %r not in %r' % (id_agent, only_agents)
                raise ResourceManager.CannotProvide(msg)
        return jobs_parallel_learning_concurrent_reps(context, data_central,
                                      id_agent, id_robot, episodes, n=n,
                                      max_reps=max_reps)
        
    rm.set_resource_provider(RM_AGENT_LEARN, rp_learn)
