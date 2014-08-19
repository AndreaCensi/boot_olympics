from .generation import for_all_sets, for_all_sets_dynamic
import os

@for_all_sets
def check_set(id_set, bootset):
    pass

@for_all_sets_dynamic
def check_set_run(context, id_set, bootset):
    params = bootset['code'][1]
    
    if not 'explore' in params:
        params['explore'] = {}
        params['explore']['num_episodes'] = 2
        params['explore']['episodes_per_tranche'] = 1
        params['explore']['max_episode_len'] = 2.0
        params['explore']['num_episodes_videos'] = 1        
    else:
        params['explore']['num_episodes'] = 2
        params['explore']['episodes_per_tranche'] = 1
        params['explore']['max_episode_len'] = 2.0
        params['explore']['num_episodes_videos'] = 1
        
    if 'servo' in params:
        if params['servo']['num_episodes'] > 0:
            params['servo']['num_episodes'] = 2
            params['servo']['episodes_per_tranche'] = 1
            params['servo']['max_episode_len'] = 2.0
            params['servo']['num_episodes_videos'] = 1
    
    if 'servonav' in params:
        if params['servonav']['num_episodes'] > 0:
            params['servonav']['num_episodes'] = 2
            params['servonav']['episodes_per_tranche'] = 1
            params['servonav']['max_episode_len'] = 2.0
            params['servonav']['num_episodes_videos'] = 1

    if 'explore_modulus' in params:
        del params['explore_modulus']

    out = os.path.join(context.get_output_dir(), 'data_central')
    from boot_manager import DataCentral
    data_central = DataCentral(out)
    from boot_manager.programs.manager.batch.main import batch_set
    batch_set(context, data_central, id_set, bootset)
    
    