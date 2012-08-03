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
            :param init_data: dictionary with fields
            
            Fields:
            - data_central
            - id_agent
            - id_robot 
        """

    @abstractmethod
    def update(self, update_data):
        """ 
            Called during interaction.
            
            :param update_data: dictionary with following fields:
            
            Fields:
            - agent
            - robot
            - obs
            
            Optional:
            - predict 
        """
        pass

    @abstractmethod
    def finish(self):
        pass
