from quickapp_boot.jobs import (jobs_parallel_learning,
    jobs_parallel_learning_concurrent)
import multiprocessing

__all__ = ['recipe_agentlearn_by_parallel',
           'recipe_agentlearn_by_parallel_concurrent']

def recipe_agentlearn_by_parallel(context, data_central, episodes):
    """
        agent-learn (id_agent, id_robot)
        
        learns all episodes for the robot in the db
    """

    rm = context.get_resource_manager()
    
    def rp_learn(context, id_agent, id_robot):
        return jobs_parallel_learning(context, data_central,
                                      id_agent, id_robot, episodes)
        
    rm.set_resource_provider('agent-learn', rp_learn)


def recipe_agentlearn_by_parallel_concurrent(context, data_central, episodes):
    """
        agent-learn (id_agent, id_robot)
        
        learns all episodes for the robot in the db
    """
    
    n = multiprocessing.cpu_count()

    rm = context.get_resource_manager()
    
    def rp_learn(context, id_agent, id_robot):
        return jobs_parallel_learning_concurrent(context, data_central,
                                      id_agent, id_robot, episodes, n=n)
        
    rm.set_resource_provider('agent-learn', rp_learn)
