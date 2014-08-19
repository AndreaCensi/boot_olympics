from .batch_video import jobs_add_videos
from .constants import default_expl_videos
from .utils import episode_id_exploration, get_tranches
from boot_manager.meat.log_learn import rawlog_from_episode
from boot_manager.meat.simulate_imp import simulate_agent_robot
from conf_tools import SemanticMistake
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
                         job_id='explore-%sof%s' % 
                                ( t + 1,
                                 len(episodes_tranches)))

        tranches.append(tranche)

        for id_episode in id_episodes:
            rawlog = context.comp(rawlog_from_episode, data_central=data_central,
                                  id_robot=id_robot, id_episode=id_episode,
                                  extra_dep=[tranche])
            episode2job[id_episode] = rawlog

    jobs_add_videos(context=context, data_central=data_central,
                    id_agent=explorer, id_robot=id_robot,
                    id_episodes=id_episodes_with_extra, 
                    videos=videos,
                    episode2job=episode2job)

    return episode2job

#     
# class BootStreamEpisodeAsRawlog(IteratorSource):
#     
#     def __init__(self, data_central, id_robot, id_episode):
#         self.data_central = data_central
#         self.id_robot = id_robot
#         self.id_episode = id_episode
#         
#     
#  
# class BootStreamAsSource(IteratorSource):
# 
#     def __init__(self, stream, to_learn):
#         self.stream = stream
#         self.to_learn = to_learn
#  
#     def get_iterator(self):
#         for x in self.stream.read(only_episodes=self.to_learn):
#             check_timed_named(x)
#             (_, (signal, _)) = x
#             yield x #obs['timestamp'], obs
#             
