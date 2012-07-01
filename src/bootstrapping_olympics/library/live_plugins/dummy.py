from bootstrapping_olympics.interfaces import LivePlugin

__all__ = ['Dummy']


class Dummy(LivePlugin):

    def __init__(self, param):
        pass

    def init(self, init_data):
        init_data.data_central
        init_data.id_agent
        init_data.id_robot

    def update(self, update_data):
        update_data.agent
        update_data.robot
        update_data.obs
        #print('observation: %s' % update_data.obs.dtype)

    def finish(self):
        pass
