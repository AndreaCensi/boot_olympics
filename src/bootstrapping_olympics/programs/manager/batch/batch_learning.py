from .utils import episode_id_exploration, get_tranches
from bootstrapping_olympics.programs.manager.meat.log_learn import learn_log
from bootstrapping_olympics.programs.manager.meat.publish_output import (
    publish_once)
from compmake import Promise
from contracts import contract

__all__ = ['jobs_learning']


@contract(simepisode2job='dict(str:isinstance(Promise))', returns=Promise)
def jobs_learning(context, data_central, id_robot, 
                  id_agent, num_ep_expl, explorer,  # XXX
                  simepisode2job,
                  publish_progress=False, save_pickle=False,
                  episodes_per_tranche=500):
    all_id_episodes = [episode_id_exploration(explorer, i)
                       for i in range(num_ep_expl)]

    previous_state = None
    tranches = get_tranches(all_id_episodes, episodes_per_tranche)
    for t, id_episodes in enumerate(tranches):
        reset = (t == 0)

        extra_dep = set()
        for id_episode in id_episodes:
            extra_dep.add(simepisode2job[id_episode])
        extra_dep = list(extra_dep)
        
        if previous_state is not None:
            extra_dep.append(previous_state)

        job_id = ('learn-%s-%s-%sof%s' % 
                  (id_robot, id_agent, t + 1, len(tranches)))
        previous_state = context.comp_config(learn_log,
                                           data_central=data_central,
                                          id_agent=id_agent,
                                          id_robot=id_robot,
                                          reset=reset,
                                          episodes=id_episodes,
                                          publish_interval=None,
                                          publish_once=False,
                                          interval_save=300,
                                          interval_print=30,
                                          extra_dep=extra_dep,
                                          job_id=job_id)

        if publish_progress:
            context.comp_config(publish_once, data_central, id_agent, id_robot,
                 phase='learn', progress='t%03d' % t,
                 job_id='report-learn-%s-%s-%s' % (id_robot, id_agent, t),
                 extra_dep=previous_state)

    context.comp_config(publish_once, data_central, id_agent, id_robot,
         phase='learn', progress='all', save_pickle=save_pickle,
         job_id='report-learn-%s-%s' % (id_robot, id_agent),
         extra_dep=[previous_state])

    return previous_state
