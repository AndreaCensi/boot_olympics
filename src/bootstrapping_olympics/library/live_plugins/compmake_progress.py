from bootstrapping_olympics import LivePlugin


__all__ = ['CompmakeProgress']


class CompmakeProgress(LivePlugin):
    
    def init(self, data):
        pass
    
    def update(self, up):
        progress = up['progress']
        if progress.obs.done % 200 == 0:
            from compmake import progress as compmake_progress
            compmake_progress('Observations', (progress.obs.done, progress.obs.target))

