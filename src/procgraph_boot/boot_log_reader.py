from blocks import check_timed_named
from procgraph import Block, IteratorGenerator


__all__ = ['BOLogReader']


class BOLogReader(IteratorGenerator):
    '''
        Reads BootOlympics logs into ProcGraph.
    '''
    Block.alias('boot_log_reader')
    Block.output('observations')
    Block.output('commands')
    Block.output('extra')
    
    Block.config('logdir', 'BootOlympics logdir')
    Block.config('id_robot', 'Name of the robot')
    Block.config('id_agent', 'If given, selects those for the agent',
                 default="")
    Block.config('id_episode', 'If given, select a specific episode',
                 default="")
    Block.config('read_extra', 'Load the extra information.', default=True)

    def init_iterator(self):
        from boot_manager import LogIndex
        index = LogIndex()
        index.index(self.config.logdir)

        id_robot = self.config.id_robot
        id_agent = self.config.id_agent
        id_episode = self.config.id_episode
        read_extra = self.config.read_extra

        use_signals = ['commands','observations']
        if read_extra:
            use_signals.append('extra')

        self.info('Reading logs for robot: %r agent: %r episodes: %r' % 
                (id_robot, id_agent, id_episode))
        self.info('read extra: %r' % read_extra)

        if not id_episode:
            self.info('Reading all episodes for %r/%r' % 
                      (id_robot, id_agent))

            if id_agent is not None:
                # Checking there are some episodes
                episodes = index.get_episodes_for_robot(id_robot=id_robot,
                                                        id_agent=id_agent)
                if not episodes:
                    msg = ('No episodes found for %r/%r' % 
                           (id_robot, id_agent))
                    raise Exception(msg)

            # FIXME: check here if there is robot_state in the extra
            for x in index.read_all_robot_streams(id_robot=id_robot,
                                                    id_agent=id_agent,
                                                    read_extra=read_extra):
                check_timed_named(x)
                t, (signal, value) = x
                if signal in use_signals:
                    yield  signal, t, value # here we have differnet convention
                
        else:
            for x in index.read_robot_episode(id_robot=id_robot,
                                                id_episode=id_episode,
                                                read_extra=read_extra):
                check_timed_named(x)
                t, (signal, value) = x
                if signal in use_signals:
                    yield  signal, t, value # here we have differnet convention


    def finish(self):
        pass
