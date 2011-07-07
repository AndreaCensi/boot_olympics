from ..interfaces import AgentInterface
import numpy as np
from time import time

class SimpleAgent(AgentInterface):
    
    	def perform_task(self, task):
        	return False
        
    	def init(self, sensels_shape, commands_spec, dict_args):
		self.beta = dict_args['beta']
		self.num_sensels = len(sensels_shape)        
		self.num_commands = len(commands_spec)
		self.max_command = commands_spec[0][1]
		self.Ey = np.zeros(self.num_sensels)
		self.Ey_hat = np.zeros(self.num_sensels)
		self.last_sensel_values = np.zeros(self.num_sensels)
		self.Eu = np.zeros(self.num_commands)
		self.time_of_switch = 0    
		self.time_last_obs = 0
		self.T = 0
		self.u = np.zeros(self.num_commands)
		self.time_of_init = time()
		self.time_last_print = 0

    	def process_observations(self, observations):
		self.Ey = (self.Ey * (self.time_last_obs - self.time_of_init) + np.array(observations) * (time() - self.time_last_obs)) / (time() - self.time_of_init) 

		#It seems that the quote get a false value when dividing by 'dt'	
		y_hat = (np.array(observations) - self.last_sensel_values) / (time() - self.time_last_obs)			
		
		self.Ey_hat = (self.Ey_hat * (self.time_last_obs - self.time_of_init) + y_hat * (time() - self.time_last_obs)) / (time() - self.time_of_init) 
		
		self.time_last_obs = time()
		self.last_sensel_values = observations 				
		if int(self.time_last_obs) % 2 == 0 and int(self.time_last_obs) != self.time_last_print:		
			self.print_averages()
			self.time_last_print = int(self.time_last_obs)

	def print_averages(self):	
		print "Counter: %8d E{y} = ["%(self.time_last_obs),
		for i in range(0,self.num_sensels): 
			print "%7.3f"%(self.Ey[i]),
		print "], E{y_hat} = [",			
		for i in range(0, self.num_sensels):
			print"%7.3f"%(self.Ey_hat[i]),
		print "] E{u} = %7.3f"%(self.Eu)    


    	def choose_commands(self):
		self.update_commands()
		return self.u

	def update_commands(self):
		if (time() - self.time_of_switch >= self.T):
			self.Eu = (self.Eu * (self.time_of_switch - self.time_of_init) + self.u * (time() - self.time_of_switch)) / (time() - self.time_of_init)        
			T = np.random.exponential(self.beta, 1)
			self.u = np.array(np.random.uniform(-self.max_command, self.max_command,self.num_commands))
			self.time_of_switch = time()
						




