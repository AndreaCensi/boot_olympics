from . import  logger
from bootstrapping_olympics.utils import InAWhile
from compmake import progress as compmake_progress
from bootstrapping_olympics.interfaces.live_plugin import LivePlugin


class CompmakeProgress(LivePlugin):
    
    def init(self, data):
        pass
    
    def update(self, up):
        progress = up['progress']
        if progress.obs.done % 200 == 0:
            compmake_progress('Observations', (progress.obs.done, progress.obs.target))


class PrintStatus(LivePlugin):
    
    def __init__(self, interval_print):
        self.tracker = InAWhile(interval_print)

    def init(self, data):
        pass

    def starting_stream(self, stream):
        """ A new stream is being read """
        self.cur_stream_observations = 0
        self.stream = stream
    
    def update(self, up):
        self.cur_stream_observations += 1
        progress = up['progress']
        state = up['state']
        
        if not self.tracker.its_time():
            return

        perc_obs = 100 * (float(progress.obs.done) / progress.obs.target)
        perc_obs_log = 100 * (float(self.cur_stream_observations) / self.stream.get_num_observations())
        msg = ('overall %.2f%% (log %3d%%) (eps: %4d/%d, obs: %4d/%d);'
               ' %5.1f fps' % 
               (perc_obs, perc_obs_log, len(state.id_episodes),
                 progress.eps.target, state.num_observations, progress.obs.target,
                self.tracker.fps()))
        logger.info(msg)


# 
#             if publish_interval is not None:
#                 if 0 == state.num_observations % publish_interval:
#                     phase = 'learn-active'
#                     filename = ds.get_report_filename(id_agent=id_agent, id_robot=id_robot,
#                                                       id_state=state.id_state, phase=phase)
# 
#                     publish_agent_output(data_central, state, agent, '%05d' % state.num_observations,
#                                          filename)
