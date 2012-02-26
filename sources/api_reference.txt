.. _api_reference:

BootOlympics public API reference
================================

.. py:module:: bootstrapping_olympics


Main interfaces and related data
--------------------------------

.. autosummary::  
   :toctree: api
   
	bootstrapping_olympics.AgentInterface            
	bootstrapping_olympics.RepresentationNuisance     
	bootstrapping_olympics.RobotInterface      
	bootstrapping_olympics.BootSpec                  
	bootstrapping_olympics.StreamSpec                 
	bootstrapping_olympics.Publisher            
	

Data
^^^^^^^^^^

.. autosummary::  
   :toctree: api

    bootstrapping_olympics.boot_observations_dtype   
    bootstrapping_olympics.boot_observations_version  
	bootstrapping_olympics.RobotObservations
    bootstrapping_olympics.EpisodeDesc                
    bootstrapping_olympics.ObsKeeper                  
  
Exceptions
^^^^^^^^^^

.. autosummary::  
   :toctree: api

    bootstrapping_olympics.BootInvalidValue          
    bootstrapping_olympics.NuisanceNotInvertible    
    bootstrapping_olympics.UnsupportedSpec            
  


           

Streamels functions
--------------------- 


.. autosummary::  
   :toctree: api
   
 	bootstrapping_olympics.check_valid_streamels     
    bootstrapping_olympics.expect_size                
    bootstrapping_olympics.streamel_dtype
    bootstrapping_olympics.ValueFormats               
    bootstrapping_olympics.new_streamels
    bootstrapping_olympics.check_valid_bounds         
    bootstrapping_olympics.only_one_value
    bootstrapping_olympics.check_valid_streamels      
    bootstrapping_olympics.set_streamel_range
    bootstrapping_olympics.expect_one_of              
    bootstrapping_olympics.streamel_array


Library: agents
----------------------------------

.. automodule:: bootstrapping_olympics.examples.agents
   :members:
   :undoc-members:

Library: robots
----------------------------------

.. automodule:: bootstrapping_olympics.examples.robots
   :members:
   :undoc-members:


Library: representation nuisances
----------------------------------

.. automodule:: bootstrapping_olympics.examples.rep_nuisances
   :members:
   :undoc-members:

.. toctree::
   :hidden:
   
   bootstrapping_olympics.examples.rep_nuisances
   bootstrapping_olympics.examples.robots
   bootstrapping_olympics.examples.agents
   