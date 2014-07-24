from .batch_video import jobs_add_videos
from .utils import episode_id_exploration, get_tranches
from bootstrapping_olympics.programs.manager.batch import default_expl_videos
from bootstrapping_olympics.programs.manager.meat.simulate_imp import (
    simulate_agent_robot)
from conf_tools.exceptions import SemanticMistake
from contracts import contract

__all__ = ['jobs_tasks_explore']


@contract(returns='dict(str:isinstance(Promise))')
def jobs_tasks_explore(context,  data_central,
                        id_robot, explorer,
                            num_episodes,
                            episodes_per_tranche=10,
                            num_episodes_videos=1,
                            videos=default_expl_videos,
                            max_episode_len=10):
    """ Returns the episodes id """

    if num_episodes_videos > num_episodes:
        msg = ('Requested %d videos for only %d episodes' % 
               (num_episodes_videos, num_episodes))
        raise SemanticMistake(msg)

    # Divide the simulation in parallel tranches
    all_id_episodes = [episode_id_exploration(explorer, i)
                       for i in range(num_episodes)]

    # These are episodes for which we want to save extra information
    id_episodes_with_extra = [episode_id_exploration(explorer, i)
                              for i in range(num_episodes_videos)]

    tranches = []

    episodes_tranches = get_tranches(all_id_episodes, episodes_per_tranche)
    
    episode2job = {}
    for t, id_episodes in enumerate(episodes_tranches):
        write_extra = len(set(id_episodes) & 
                          set(id_episodes_with_extra)) > 0

        tranche = context.comp_config(simulate_agent_robot,
                        data_central=data_central,
                         id_agent=explorer,
                         id_robot=id_robot,
                         max_episode_len=max_episode_len,
                         stateful=False,
                         id_episodes=id_episodes,
                         cumulative=False,
                         write_extra=write_extra,
                         job_id='explore-%s-%s-%sof%s' % 
                                (id_robot, explorer, t + 1,
                                 len(episodes_tranches)))

        tranches.append(tranche)

        for id_episode in id_episodes:
            episode2job[id_episode] = tranche

    jobs_add_videos(context=context, data_central=data_central,
                    id_agent=explorer, id_robot=id_robot,
                    id_episodes=id_episodes_with_extra, 
                    videos=videos,
                    episode2job=episode2job)

    return episode2job

