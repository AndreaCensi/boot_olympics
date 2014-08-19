from boot_manager import DataCentral
from bootstrapping_olympics import get_boot_config
from contracts import contract
from quickapp import CompmakeContext
import itertools
from boot_manager.meat.video import create_video

__all__ = ['jobs_add_videos']


@contract(context=CompmakeContext,
          data_central=DataCentral,
          episode2job='dict(str:isinstance(Promise))',
          returns='dict(str:isinstance(Promise))')
def jobs_add_videos(context, data_central, id_agent, id_robot, id_episodes, videos,
                    episode2job):
    # todo: temporary videos
    res = {}
    for id_episode, id_video in itertools.product(id_episodes, videos):
        config = get_boot_config()
        
        code_spec = config.specs['videos'][id_video]
        model = code_spec['code'][0]
        model_params = code_spec['code'][1]

        extra_dep = episode2job[id_episode]

        res[id_episode] = context.comp_config(create_video,
             data_central=data_central,
             id_episode=id_episode,
             id_agent=id_agent,
             id_robot=id_robot,
             suffix=id_video,
             model=model,
             model_params=model_params,
             job_id='video-%s-%s' % (id_episode, id_video),
             extra_dep=extra_dep)
    return res