.. _api_reference:

BootOlympics public API reference
================================

.. py:module:: bootstrapping_olympics


Main interfaces and related data
--------------------------------

.. autosummary::  
   :toctree: api
   
	AgentInterface            
	RepresentationNuisance     
	RobotInterface      
	BootSpec                  
	StreamSpec     
	
The following are ...

.. autosummary::  
   :toctree: api

	boot_observations_dtype   
	RobotObservations
	EpisodeDesc                
	Publisher            

  
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

.. include:: api/agents.rst.inc

Library: robots
----------------------------------

.. include:: api/robots.rst.inc

Library: representation nuisances
----------------------------------

.. include:: api/nuisances.rst.inc
