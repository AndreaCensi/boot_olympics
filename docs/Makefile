github_repo=git@github.com:AndreaCensi/boot_olympics.git
	
include docs.mk

generate-custom:
	./list_stuff.py -i bootstrapping_olympics.RepresentationNuisance bootstrapping_olympics.library.nuisances > source/api/nuisances.rst.inc
	./list_stuff.py -i bootstrapping_olympics.AgentInterface bootstrapping_olympics.library.agents > source/api/agents.rst.inc
	./list_stuff.py -i bootstrapping_olympics.RobotInterface bootstrapping_olympics.library.robots > source/api/robots.rst.inc
	