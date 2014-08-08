from .utils import get_tranches
from bootstrapping_olympics import logger
from compmake import Context, Promise
from contracts import contract
from contracts.utils import check_isinstance
from rawlogs import RawLog
from rawlogs.library import get_log_parts
from streamels import BootSpec
import warnings

__all__ = ['jobs_learning'] 
            
@contract(simepisode2job='dict(str:isinstance(Promise))', 
          episodes_per_tranche='int,>=1',
          max_slice_len='float,>0',
          parallel_hint='bool',
          more_phases='bool',
          boot_spec=BootSpec,
          returns=Promise)
def jobs_learning(context, 
                  boot_spec, 
                  id_agent,
                  simepisode2job,
                  episodes_per_tranche,
                  max_slice_len, # maximum length for a slice of the log
                  parallel_hint, # none to deactivate or (i, n)
                  more_phases, # if True, do more phases
                  publish_progress=False, 
                  save_pickle=False):
    """ simepisode[id_episode] must be a promise to a Rawlog. """

    if parallel_hint:
        logger.error('parallel_hint hint not implemented yet.')
    
    if more_phases:
        logger.error('more_phases not implemented yet.')

    agent_states = []
    tranches = get_tranches(list(simepisode2job), episodes_per_tranche)
    for t, id_episodes in enumerate(tranches):
        c2 = context.child('tr%s-of-%s' % (t + 1, len(tranches))) 
        episodes_for_this_one = {}
        for id_episode in id_episodes:
            episodes_for_this_one[id_episode] = simepisode2job[id_episode]
            
        state_tranche = c2.comp_config_dynamic(jobs_learning_logs,
                                               id_agent=id_agent, 
                                               logs=episodes_for_this_one,
                                               max_slice_len=max_slice_len,
                                               boot_spec=boot_spec)

        agent_states.append(state_tranche)
        
        warnings.warn('redo this')
        
        if publish_progress:
            logger.error('TODO: implement publish')
#         
#         if publish_progress:
#             context.comp_config(publish_once, data_central, id_agent, id_robot,
#                  phase='learn', progress='t%03d' % t,
#                  job_id='report-learn-%s-%s-%s' % (id_robot, id_agent, t),
#                  extra_dep=previous_state)
# 
    logger.error('TODO: implement publish')
#     context.comp_config(publish_once, data_central, id_agent, id_robot,
#          phase='learn', progress='all', save_pickle=save_pickle,
#          job_id='report-learn-%s-%s' % (id_robot, id_agent),
#          extra_dep=[previous_state])

    agent_state = merge_agents_recursive(context, agent_states)
    
    return agent_state

@contract(context=Context, 
          logs='dict(str:isinstance(RawLog))',
          max_slice_len='float,>0',
          id_agent=str,
          boot_spec=BootSpec)
def jobs_learning_logs(context, logs, id_agent, max_slice_len,
                       boot_spec=BootSpec):
    check_isinstance(logs, dict)
    for id_episode, rawlog in logs.items():
        check_isinstance(rawlog, RawLog)
        
    from bootstrapping_olympics.programs.manager.meat.log_learn import learn_rawlog

    agent_states = []
    for id_part, part in get_log_tranches(logs, max_slice_len).items():
        agent_state = context.comp_config(learn_rawlog, 
                                          rawlog=part,
                                          id_agent=id_agent,
                                          boot_spec=boot_spec,
                                          id_episode=id_part) 
        agent_states.append(agent_state)
    
    agent_state = merge_agents_recursive(context, agent_states)
    
    return agent_state

     
@contract(logs='dict(str:isinstance(RawLog))',
          returns='dict(str:isinstance(RawLog))')
def get_log_tranches(logs, tranche_length_sec):
    res = {}
    for id_log, log in logs.items():
        print('Computing for %s / %s (length: %s)' % (id_log, log, tranche_length_sec))
        parts = get_log_parts(log, tranche_length_sec)
        for id_part, part in parts.items():
            name = '%s-%s' % (id_log, id_part)
            res[name] = part
    return res


@contract(context=Context, agents='list[>=1](isinstance(Promise))', returns=Promise)
def merge_agents_recursive(context, agents):
    """ merges hierarchically """
    n = len(agents)
    if n == 1:
        return agents[0]
    
    half = n / 2
    a1 = agents[0:half]
    a2 = agents[half:]
    assert len(a1) > 0
    assert len(a2) > 0
    
    m1 = merge_agents_recursive(context, a1)
    m2 = merge_agents_recursive(context, a2)
    return context.comp(merge_agents, m1, m2)
      
    
def merge_agents(as1, as2):
    agent1, state1 = as1
    agent2, state2 = as2
    warnings.warn('Todo: check not overlapping')
    print('merging agent 1: %s obs' % state1.num_observations)
    print('merging agent 2: %s obs' % state2.num_observations)
    agent1.merge(agent2)
    state1.merge(state2)
    print('res: %s obs' % state1.num_observations)
    return agent1, state1