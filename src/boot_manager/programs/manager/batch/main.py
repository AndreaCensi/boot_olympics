from .batch_explore import jobs_tasks_explore
from .batch_learning import jobs_learning
from .batch_predict import jobs_tasks_predict
from .batch_repor import jobs_task_robot_report
from .batch_servo import jobs_tasks_servo
from .batch_servonav import jobs_tasks_servonav
from .utils import are_compatible, instance_agent, instance_robot
from boot_manager import DataCentral, get_conftools_bootbatchsets
from boot_manager.meat.log_learn import get_robot_boot_spec
from bootstrapping_olympics import (BasicAgent, BasicRobot, PredictingAgent, 
    ServoingAgent, logger)
from bootstrapping_olympics.utils import safe_makedirs, safe_symlink
from compmake import Context, Promise
from conf_tools import ConfToolsException, SemanticMistake, import_name
from contracts import contract
from pprint import pformat
from quickapp import iterate_context_names
import os


def batch_process_manager(context, data_central, which_sets): 
    sets_config = get_conftools_bootbatchsets() 

    which_sets_int = sets_config.expand_names(which_sets) 

    for x in which_sets_int:
        sets_config[x] 
        
    if len(which_sets_int) == 1:
        combid = which_sets[0]
    else:
        combid = '-'.join(which_sets)

    # Create the new root        
    root = data_central.root
    root_set = os.path.join(data_central.root, 'sets', combid)
    safe_makedirs(root_set)
    data_central_set = DataCentral(root_set)

    # add symbolic links to logs and config
#     main_config = os.path.realpath(os.path.join(root, 'config'))
#     set_config = os.path.join(root_set, 'config')
#     safe_symlink(main_config, set_config) 

    safe_makedirs(os.path.join(root_set, 'logs'))
    safe_symlink(os.path.join(root, 'logs'),
                 os.path.join(root_set, 'logs', 'original'))

    print('id_sets: %s' % which_sets_int)
    for c, id_set in iterate_context_names(context, which_sets_int):
        print('set %r'% id_set)
        try:
            spec = sets_config[x]
            batch_set(c, data_central_set, id_set, spec)
        except ConfToolsException:
            msg = ('Bad configuration for the set %r with spec\n %s' % 
                   (id_set, pformat(spec)))
            logger.error(msg)
            raise 

def batch_set(context, data_central, id_set, spec):  # @UnusedVariable
    function_name = spec['code'][0]
    args = spec['code'][1]
    function = import_name(function_name)
    function(context=context, data_central=data_central, **args)




        
def batch_jobs1(context, data_central, robots, agents,
                explore,
                learning=None,
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
                             explorer=explorer, 
                             simepisode2jobid=episode2jobid,
                             # Tasks params
                             servo=servo, 
                             learning=learning,
                             servonav=servonav, 
                             predict=predict,
                             )


    
@contract(context=Context,  data_central=DataCentral, 
          id_agent=str, agent=BasicAgent,
          id_robot=str, robot=BasicRobot, 
          simepisode2jobid='dict(str:str)',
          servo='None|dict', servonav='None|dict', predict='None|dict')
def jobs_agent_robot(context, data_central, id_agent, agent, id_robot, robot,
                     explorer, simepisode2jobid,
                     learning=None,
                     servo=None, servonav=None, predict=None):

    simepisode2job=dict([(e, Promise(x)) for e,x in simepisode2jobid.items()])

    
    compatible, reason = are_compatible(robot=robot, agent=agent)
    if not compatible:
        logger.info('Avoiding combination %s / %s: %s' % 
                     (id_robot, id_agent, reason))
        return

    if learning is None:
        learning = {}
        learning['max_slice_len'] = 500 # maximum length for a slice of the log
        learning['parallel_hint'] = True # none to deactivate or (i, n)
        learning['more_phases'] = True
        learning['episodes_per_tranche'] = 10
        logger.error('Learning parameters not passed --- using defaults: %s' % learning)

    

    boot_spec = get_robot_boot_spec(data_central, id_robot)
                  
    agent_state = jobs_learning(context=context,
                                      boot_spec=boot_spec,
                                      id_agent=id_agent,                                
                                      simepisode2job=simepisode2job ,
                                      episodes_per_tranche = learning['episodes_per_tranche'],
                                      max_slice_len = learning['max_slice_len'],
                                      parallel_hint = learning['parallel_hint'],
                                      more_phases = learning['more_phases'],
                                      publish_progress=False,
                                      save_pickle=True)
    
    agent_has_learned = context.comp(save_state, 
                                     data_central=data_central,
                                     id_agent=id_agent, 
                                     id_robot=id_robot, 
                                     agent_state=agent_state)

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
    

@contract(data_central=DataCentral, id_agent='str', id_robot='str')
def save_state(data_central, id_agent, id_robot, agent_state):
    agent, state = agent_state
    state.agent_state = agent.get_state()
    db = data_central.get_agent_state_db() 
    db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
    return agent_state
