from quickapp_boot.jobs import jobs_parallel_learning

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
