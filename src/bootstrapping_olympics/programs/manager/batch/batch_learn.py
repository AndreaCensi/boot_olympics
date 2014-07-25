'''Some functions to help in writing experiments scripts'''

from .batch_explore import jobs_tasks_explore
from .batch_learning import jobs_learning
from .batch_predict import jobs_tasks_predict
from .batch_repor import jobs_task_robot_report
from .batch_servo import jobs_tasks_servo
from .batch_servonav import jobs_tasks_servonav
from .utils import are_compatible, instance_agent, instance_robot
from bootstrapping_olympics import (BasicAgent, BasicRobot, PredictingAgent, 
    ServoingAgent, logger)
from bootstrapping_olympics.programs.manager.meat.data_central import (
    DataCentral)
from compmake import Context
from compmake.structures import Promise
from conf_tools import SemanticMistake
from contracts import contract
from quickapp import iterate_context_names

        
def batch_jobs1(context, data_central, robots, agents,
                explore,
                    servo=None,
                    servonav=None,
                    predict=None):
    if not robots:
        raise SemanticMistake('Please specify at least one robot.')

    if not agents:
        raise SemanticMistake('Please specify at least one agent.')
        
    print('robots: %r ' % robots)
    print('agents: %r ' % agents)
    
    agent2instance = {}
    for id_agent in agents:
        agent2instance[id_agent] = context.comp_config(instance_agent, id_agent,
                                                       job_id='instance-%s'%id_agent) 
        
    for c, id_robot in iterate_context_names(context, robots, key='id_robot'):
        jobs_task_robot_report(c, id_robot=id_robot)
        robot = c.comp_config(instance_robot, id_robot)
        episode2job = jobs_tasks_explore(context=c,
                                         data_central=data_central,
                                         id_robot=id_robot, 
                                         **explore)
    
        num_ep_expl=explore['num_episodes']
        explorer=explore['explorer']
        for c2, id_agent in iterate_context_names(c, agents, key='id_agent'):
            agent = agent2instance[id_agent]
            # hack: we don't want to resolve the references
            episode2jobid = dict([(episode, x.job_id)
                                  for episode, x in episode2job.items()])
            c2.comp_config_dynamic(jobs_agent_robot,
                             data_central=data_central,
                             id_agent=id_agent, id_robot=id_robot,
                             agent=agent, robot=robot,
                             num_ep_expl=num_ep_expl,
                             explorer=explorer, 
                             servo=servo, 
                             servonav=servonav, predict=predict,
                             simepisode2jobid=episode2jobid)


    
@contract(context=Context,  data_central=DataCentral, 
          id_agent=str, agent=BasicAgent,
          id_robot=str, robot=BasicRobot, num_ep_expl='>=1',
          simepisode2jobid='dict(str:str)',
          servo='None|dict', servonav='None|dict', predict='None|dict')
def jobs_agent_robot(context, data_central, id_agent, agent, id_robot, robot,
                     num_ep_expl, explorer, simepisode2jobid,
                     servo=None, servonav=None, predict=None):

    simepisode2job=dict([(e, Promise(x)) for e,x in simepisode2jobid.items()])

    
    compatible, reason = are_compatible(robot=robot, agent=agent)
    if not compatible:
        logger.info('Avoiding combination %s / %s: %s' % 
                     (id_robot, id_agent, reason))
        return

    agent_has_learned = jobs_learning(context=context,
                                      data_central=data_central,
                                      id_robot=id_robot, 
                                      id_agent=id_agent, 
                                      num_ep_expl=num_ep_expl,
                                      explorer=explorer,
                                      publish_progress=False,
                                      save_pickle=True,
                                      simepisode2job=simepisode2job)

    if servo is not None:
        if isinstance(agent, ServoingAgent):
            jobs_tasks_servo(context=context,data_central=data_central,
                              id_agent=id_agent, id_robot=id_robot,
                              agent_has_learned=agent_has_learned,
                              **servo)

    if servonav is not None:
        if isinstance(agent, ServoingAgent):
            jobs_tasks_servonav(context=context,data_central=data_central,
                                            id_agent=id_agent, id_robot=id_robot,
                                            agent_has_learned=agent_has_learned,
                                            **servonav)

    if predict is not None:
        if isinstance(agent, PredictingAgent):
            jobs_tasks_predict(context=context,data_central=data_central,
                                id_agent=id_agent, id_robot=id_robot,
                                agent_has_learned=agent_has_learned,
                                **predict)
    
