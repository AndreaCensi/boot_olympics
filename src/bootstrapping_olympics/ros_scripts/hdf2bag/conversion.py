from procgraph import simple_block, register_model_spec, COMPULSORY, TIMESTAMP
import numpy as np
from procgraph.core.block import Block

register_model_spec('''
--- model hdf2bag_conversion
config hdf 
config bag 
config id_robot
config id_actuators
config id_sensors
config id_episode
config id_environment
config commands_spec

import procgraph_hdf
import procgraph_ros

|log:hdfread file=$hdf|

#log.y --> |info|
#log.u --> |info|

log.y[y], log.u[u] -> |r:raw2boot| -> observations -> |bagwrite file=$bag| 

r.id_robot = $id_robot
r.id_actuators = $id_actuators
r.id_sensors = $id_sensors
r.id_episode = $id_episode
r.id_environment = $id_environment
r.commands_spec = $commands_spec
''')


class Raw2Boot(Block):
    Block.alias('raw2boot')
    Block.input('u')
    Block.input('y')
    Block.output('msg')
    
    Block.config('id_robot')
    Block.config('id_episode')
    Block.config('id_actuators')
    Block.config('id_sensors')
    Block.config('id_environment')
    Block.config('commands_spec')

    def init(self):
        self.counter = 0
    
    def update(self):
        if not self.all_input_signals_ready():
            return
        
        u = self.input.u
        y = self.input.y
        timestamp = self.get_input_timestamp(0)
        
        from bootstrapping_adapter.msg import BootstrappingObservations #@UnresolvedImport
    
        data = {
                'timestamp': timestamp,
                'counter': self.counter,
                'id_episode': self.config.id_episode,
                'sensel_values': y.reshape(y.size),
                'sensel_shape': np.array(y.shape),
                'commands': np.array(u),
                'commands_spec': self.config.commands_spec,
                'id_robot': self.config.id_robot,
                'id_actuators': self.config.id_actuators,
                'id_sensors': self.config.id_sensors,
                'id_environment': self.config.id_environment,
        }
        
        self.counter += 1
        
        msg = BootstrappingObservations(**data)

        self.set_output(0, msg, timestamp)



