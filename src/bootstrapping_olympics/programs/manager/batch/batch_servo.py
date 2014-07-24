from .batch_video import jobs_add_videos
from .utils import episode_id_servoing, get_tranches
from bootstrapping_olympics.programs.manager.batch import default_servo_videos
from bootstrapping_olympics.programs.manager.meat.data_central import (
    DataCentral)
from bootstrapping_olympics.programs.manager.meat.servo.report import (
    servo_stats_report)
from bootstrapping_olympics.programs.manager.meat.servo.servo import task_servo
from bootstrapping_olympics.programs.manager.meat.servo.summaries import (
    servo_stats_summary)
from compmake.context import Context
from conf_tools import SemanticMistake
from contracts import contract

__all__ = ['jobs_tasks_servo']

@contract(context=Context, data_central=DataCentral, id_agent='str', id_robot='str')
def jobs_tasks_servo(context, data_central, id_agent, id_robot,
                     num_episodes, agent_has_learned,
                     num_episodes_videos=0,
                     max_episode_len=10,
                     displacement=3,
                     episodes_per_tranche=1,
                     videos=default_servo_videos):
    
    if num_episodes_videos > num_episodes:
        raise SemanticMistake('More videos than episodes requested.')
    
    if num_episodes == 0:
        return 
    
    all_id_episodes = [episode_id_servoing(id_agent, i) for i in range(num_episodes)]
    id_episodes_with_extra = [
                              episode_id_servoing(id_agent, i)
                              for i in range(num_episodes_videos)]
    episode2job = {}
    summaries = []
    all_tranches = []
    episodes_tranches = get_tranches(all_id_episodes, episodes_per_tranche)
    for st, id_episodes in enumerate(episodes_tranches):
        num_episodes_with_robot_state = len(set(id_episodes) & 
                                            set(id_episodes_with_extra))
    
        tranche = context.comp_config(task_servo,
                                    data_central=data_central,
                                     id_agent=id_agent, id_robot=id_robot,
                                     max_episode_len=max_episode_len,
                                     num_episodes=len(id_episodes),
                                     id_episodes=id_episodes,
                                     cumulative=False,
                                     displacement=displacement,
                                     interval_print=5,
                                     num_episodes_with_robot_state=num_episodes_with_robot_state,
         job_id='servo-%s-%s-%sof%s' % (id_robot, id_agent, st + 1,
                                         len(episodes_tranches)),
         extra_dep=agent_has_learned)
    
        all_tranches.append(tranche)
        
        for id_episode in id_episodes:
            episode2job[id_episode] = tranche

        for id_episode in id_episodes:
            job_id = ('servo-%s-%s-%s-summary' % 
                      (id_robot, id_agent, id_episode))
            summary = context.comp_config(servo_stats_summary,
                                        data_central, id_agent,
                                        id_robot, id_episode,
                                        job_id=job_id,
                                        extra_dep=[tranche])
            summaries.append(summary)
    
    context.comp_config(servo_stats_report, data_central, id_agent,
         id_robot, summaries,
         job_id='report-servo-%s-%s' % (id_robot, id_agent))
    
    # TODO: create partial summaries
    ntot = len(summaries)
    num = ntot
    for i in range(ntot):
        num = num / 2
        if num <= 2:
            continue
        psummaries = summaries[:num]
        # logger.info('Creatint partial summaries with %d/%d using %s' % (num, ntot, psummaries))
        context.comp_config(servo_stats_report, data_central, id_agent,
                           id_robot, psummaries,
                           phase='servo_stats-partial%d' % i,
            job_id='report-servo-%s-%s-partial%d' % (id_robot, id_agent, i))
    
    jobs_add_videos(context=context,
                    data_central=data_central,
                    id_agent=id_agent,
                      id_robot=id_robot,
                      id_episodes=id_episodes_with_extra,
                      videos=videos,
                      episode2job=episode2job)
    
    return episode2job

