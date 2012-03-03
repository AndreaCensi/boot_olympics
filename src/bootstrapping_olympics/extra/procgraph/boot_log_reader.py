from procgraph import Block, Generator


class BOLogReader(Generator):
    '''
        Reads BootOlympics logs into ProcGraph.
    '''
    Block.alias('boot_log_reader')
    Block.output('observations')
    Block.config('logdir', 'BootOlympics logdir')
    Block.config('id_robot', 'Name of the robot')
    Block.config('id_agent', 'If given, selects those for the agent',
                 default="")
    Block.config('id_episode', 'If given, select a specific episode',
                 default="")
    Block.config('read_extra', 'Load the extra information.', default=True)

    def init(self):
        from bootstrapping_olympics.logs import LogIndex
        index = LogIndex()
        index.index(self.config.logdir)

        id_robot = self.config.id_robot
        id_agent = self.config.id_agent
        id_episode = self.config.id_episode
        read_extra = self.config.read_extra

        self.info('Reading logs for robot: %r agent: %r episodes: %r' %
                (id_robot, id_agent, id_episode))

        def go():
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
                for obs in index.read_all_robot_streams(id_robot=id_robot,
                                                        id_agent=id_agent,
                                                        read_extra=read_extra):
                    yield obs['timestamp'].item(), obs.copy()
                    obs['extra'] = {} # Tmp: debugging
            else:
                for obs in index.read_robot_episode(id_robot=id_robot,
                                                    id_episode=id_episode,
                                                    read_extra=read_extra):
                    yield obs['timestamp'].item(), obs.copy()
                    obs['extra'] = {} # Tmp: debugging

        self.iterator = go()

        self._load_next()

    def _load_next(self):
        try:
            self.next_timestamp, self.next_value = self.iterator.next()
            self.has_next = True
        except StopIteration:
            self.has_next = False

    def next_data_status(self):
        if self.has_next:
            return (True, self.next_timestamp)
        else:
            return (False, None)

    def update(self):
        if not self.has_next:
            return  # XXX: error here? 

        self.set_output(0, value=self.next_value,
                        timestamp=self.next_timestamp)

        self._load_next()

    def finish(self):
        pass
