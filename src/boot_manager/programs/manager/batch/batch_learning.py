from .utils import get_tranches
from boot_manager.logs.learning_state import LearningState
from boot_manager.meat.log_learn import learn_rawlog2
from bootstrapping_olympics import get_boot_config, get_conftools_agents, logger
from compmake import Context, Promise
from contracts import check_isinstance, contract
from rawlogs import RawLog
from rawlogs.library import get_log_parts
import warnings


# class LearningScheduleParams():
#     def __init__(self, patternA_slice_len,
#                        patternB_allow, 
#                        patternB_max_workers,
#                        patternD_allow,
#                        patternD_max_epochs):
#         self.patternA_slice_len = patternA_slice_len
#         self.patternB_allow = patternB_allow
#         self.patternB_max_workers = patternB_max_workers
#         self.patternD_allow = patternD_allow
#         self.patternD_max_epochs = patternD_max_epochs
    
__all__ = [
    'jobs_learning'
] 
            
@contract(simepisode2job='dict(str:isinstance(Promise))', 
          episodes_per_tranche='int,>=1',
          max_slice_len='(int|float),>0',
          parallel_hint='bool',
          more_phases='bool',
#         boot_spec=BootSpec,
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
                  ):
    """ simepisode[id_episode] must be a promise to a RawLog. """

    max_slice_len = float(max_slice_len)
    if parallel_hint:
        logger.error('parallel_hint hint not implemented yet.')
    
    # this is a function that returns the tuple agent_state
    learn_function = jobs_learning_one_phase
    # parameters in addition to first "context" parameter
    # and the agent_state0 parameter
    learn_params = dict(simepisode2job=simepisode2job, 
                        episodes_per_tranche=episodes_per_tranche,
                        max_slice_len=max_slice_len)
    agent_state0 = context.comp_config(initial_agent_state,
                                       id_agent=id_agent, boot_spec=boot_spec)
        
    should_do_more_phases = more_phases and agent_supports_phases(id_agent)
    if not should_do_more_phases:
        return learn_function(context=context, agent_state0=agent_state0, **learn_params)                 
    else:
        return jobs_learning_phases(context=context,
                                    phase=0, 
                                    agent_state0=agent_state0,
                                    learn_function=learn_function,
                                    learn_params=learn_params)
        
def agent_supports_phases(id_agent):
    config =get_conftools_agents()
    agent = config.instance(id_agent)
    return True # XXX: needs another interface

@contract(phase='int', learn_params='dict')
def jobs_learning_phases(context, phase, agent_state0,
                         learn_function, learn_params):
    c = context.child('ph%d' % phase)
    agent_state1 = c.comp_config_dynamic(learn_function, agent_state0=agent_state0, **learn_params)
    return context.comp_config_dynamic(phase_check_need_another,
                                   agent_state1, 
                                   agent_state_job_id=agent_state1.job_id,
                                   phase=phase,
                                   learn_function=learn_function, 
                                   learn_params=learn_params,
                                   job_id='phase_check_need_another-afterph%d' % phase)

def phase_check_need_another(context, agent_state, agent_state_job_id, phase, 
                             learn_function, learn_params):
    agent, _ = agent_state
    
    if agent.need_another_phase():
        agent_state0 = context.comp(phase_advance_state, Promise(agent_state_job_id))
        return context.comp_config_dynamic(jobs_learning_phases, 
                                           phase=(phase + 1),
                                           agent_state0=agent_state0,
                                           learn_function=learn_function,
                                           learn_params=learn_params,
                                           job_id='jobs_learning_phases_for_%d' % (phase+1))
    else:
        # XXX: very expensive
        # note, the need_another_phase() has side effects
        # return agent, learning_state
        return Promise(agent_state_job_id)
    
def initial_agent_state(id_agent, boot_spec):
    boot_config = get_boot_config()
    learning_state = LearningState(id_robot='unnamed', id_agent=id_agent)
    learning_state.id_episodes = set()
    learning_state.num_observations = 0

    agent = boot_config.agents.instance(id_agent)
    agent.init(boot_spec)
    return agent, learning_state




def phase_advance_state(agent_state):
    agent, learning_state = agent_state
    assert agent.need_another_phase()
    agent.start_next_phase()
    learning_state.id_episodes = set()
    learning_state.num_observations = 0
    return agent, learning_state


def jobs_learning_one_phase(context,  agent_state0,
                            simepisode2job, episodes_per_tranche,
                            max_slice_len):
    
    agent_states = []
    tranches = get_tranches(list(simepisode2job), episodes_per_tranche)
    
    print('found %d episodes' % len(simepisode2job))
    print('divided in %d tranches' % len(tranches))
    
    for t, id_episodes in enumerate(tranches):
        c2 = context.child('tr%s-of-%s' % (t + 1, len(tranches))) 
        episodes_for_this_one = {}
        for id_episode in id_episodes:
            episodes_for_this_one[id_episode] = simepisode2job[id_episode]
            
        state_tranche = c2.comp_config_dynamic(jobs_learning_logs,
                                               agent_state0=agent_state0,
                                               logs=episodes_for_this_one,
                                               max_slice_len=max_slice_len,
                                               )

        agent_states.append(state_tranche)

    agent_state = merge_agents_recursive(context, agent_states)
    
    return agent_state
    

@contract(
          logs='dict(str:isinstance(RawLog))',
          max_slice_len='float,>0',
          agent_state0='tuple[2]')
def jobs_learning_logs(context, agent_state0, 
                       logs, 
                       max_slice_len):
    check_isinstance(logs, dict)
    for id_episode, rawlog in logs.items():
        check_isinstance(id_episode, str)
        check_isinstance(rawlog, RawLog)
    
    parts = get_log_tranches(logs, max_slice_len)
    agent_states = []
    for id_part, part in parts.items():
        agent_state = context.comp_config(learn_rawlog2, 
                                          rawlog=part,
                                          agent_state0=agent_state0,
                                          id_episode=id_part,
                                          job_id='learn-%s' % id_part) 
        agent_states.append(agent_state)
    
    agent_state = merge_agents_recursive(context, agent_states)
    
    return agent_state

     
@contract(logs='dict(str:isinstance(RawLog))',
          returns='dict(str:isinstance(RawLog))')
def get_log_tranches(logs, tranche_length_sec):
    res = {}
    for id_log, log in logs.items():
        parts = get_log_parts(log, tranche_length_sec)
        print('Computing for %s / %s. Slice len: %s. n: %s' % 
              (id_log, log, tranche_length_sec, len(parts)))
        for id_part, part in parts.items():
            name = '%s-%s' % (id_log, id_part)
            res[name] = part
    return res


@contract( agents='list[>=1](isinstance(Promise))',
          returns=Promise)
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
