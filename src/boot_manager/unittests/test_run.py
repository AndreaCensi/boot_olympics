from boot_manager.unittests.generation import for_all_sets, for_all_sets_dynamic
import os
from bootstrapping_olympics.programs.manager.batch.batch_manager import batch_set
from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral

@for_all_sets
def check_set(id_set, bootset):
    pass

@for_all_sets_dynamic
def check_set_run(context, id_set, bootset):

#     print(bootset)
    params = bootset['code'][1]
    
    params['explore']['num_episodes'] = 2
    params['explore']['max_episode_len'] = 2.0
    params['explore']['num_episodes_videos'] = 1
    params['servo']['num_episodes'] = 2
    params['servo']['max_episode_len'] = 2.0
    params['servo']['num_episodes_videos'] = 1
    if params['servonav']['num_episodes'] > 0:
        params['servonav']['num_episodes'] = 2
        params['servonav']['max_episode_len'] = 2.0
        params['servonav']['num_episodes_videos'] = 1

    data_central = DataCentral(os.path.join(context.get_output_dir(), 'data_central'))
    batch_set(context, data_central, id_set, bootset)
    


# 
#     
# def report_f(id_set, bootset):
#     from reprep import Report
#     r = Report()
#     
#     r.text('id_set', id_set )
#     r.text('bootset', bootset )
#     
#     return r