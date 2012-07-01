from abc import ABCMeta, abstractmethod
from collections import namedtuple


class LivePlugin:
    __metaclass__ = ABCMeta

    """ 
        The interface for plugins that are run in parallel
        with the agent. Mostly used for visualization or 
        informal statistics.
    """

    InitData = namedtuple('InitData',
                          ['data_central',
                           'id_agent',
                           'id_robot'
                           ])

    @abstractmethod
    def init(self, init_data):
        """
            :param init_data: structure InitData 
        """

    UpdateData = namedtuple('UpdateData',
                            ['agent',
                             'robot', # Might be none if it's a log 
                             'obs'])

    @abstractmethod
    def update(self, update_data):
        """ 
            Called during interaction.
            
            :param update_data: structure UpdateData 
        """
        pass

    @abstractmethod
    def finish(self):
        pass
