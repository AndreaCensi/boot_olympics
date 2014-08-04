from .batch_video import jobs_add_videos
from .constants import default_servonav_videos
from .utils import episode_id_servonav, get_tranches
from bootstrapping_olympics.programs.manager.meat.data_central import (
    DataCentral)
from bootstrapping_olympics.programs.manager.meat.servonav.task import (
    task_servonav)
from compmake import Context
from conf_tools import SemanticMistake
from contracts import contract

__all__ = ['jobs_tasks_servonav']


@contract(context=Context, data_central=DataCentral, id_agent='str', id_robot='str')
def jobs_tasks_servonav(context, data_central, id_agent, id_robot,
                             num_episodes,
                             agent_has_learned,
                             episodes_per_tranche=1,
                             num_episodes_videos=0,
                             max_episode_len=10,
                             resolution=1,
                             fail_if_not_working=False,
                             videos=default_servonav_videos):

    if num_episodes_videos > num_episodes:
        raise SemanticMistake('More videos than episodes requested.')

    if num_episodes == 0:
        return

    all_id_episodes = [episode_id_servonav(id_agent, i)
                       for i in range(num_episodes)]
    id_episodes_with_extra = [episode_id_servonav(id_agent, i)
                       for i in range(num_episodes_videos)]

    all_tranches = []
    episodes_tranches = get_tranches(all_id_episodes,episodes_per_tranche)
    episode2job = {}
    for st, id_episodes in enumerate(episodes_tranches):
        num_episodes_with_robot_state = len(set(id_episodes) & 
                                            set(id_episodes_with_extra))

        tranche = context.comp_config(task_servonav, data_central=data_central,
         id_agent=id_agent, id_robot=id_robot,
         max_episode_len=max_episode_len,
         num_episodes=len(id_episodes),
         id_episodes=id_episodes,
         cumulative=False,
         resolution=resolution,
         interval_print=5,
         fail_if_not_working=fail_if_not_working,
         num_episodes_with_robot_state=num_episodes_with_robot_state,
         job_id='servonav-tr%sof%s' % (st + 1, len(episodes_tranches)),
         extra_dep=agent_has_learned)

        all_tranches.append(tranche)
        for id_episode in id_episodes:
            episode2job[id_episode] = tranche

    jobs_add_videos(context=context, 
                    data_central=data_central,
                    id_agent=id_agent,
                      id_robot=id_robot,
                      id_episodes=id_episodes_with_extra,
                      videos=videos,
                      episode2job=episode2job)
    
    return episode2job
