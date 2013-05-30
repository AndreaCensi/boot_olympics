from . import load_agent_state, save_report
from bootstrapping_olympics import AgentInterface
from bootstrapping_olympics.programs.manager.meat.data_central import (
    DataCentral)
from contracts import contract
from reprep import Report
import os

__all__ = ['publish_once', 'publish_agent_output', 'get_agent_report',
           'get_agentstate_report',
           'get_agent_report_from_state']

def publish_once(data_central, id_agent, id_robot,
                 phase='learn', progress='all',
                 save_pickle=False):  # TODO: 'learn' in constants
    # XXX: progress is not used so far
    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=False)
    
    ds = data_central.get_dir_structure()
    filename = ds.get_report_filename(id_agent=id_agent, id_robot=id_robot,
                                       id_state=state.id_state, phase=phase)
    res_dir = ds.get_report_res_dir(id_agent=id_agent, id_robot=id_robot,
                                    id_state=state.id_state, phase=phase)

    publish_agent_output(data_central, state, agent, progress,
                         save_pickle=save_pickle,
                         filename=filename, rd=res_dir) 


def publish_agent_output(data_central, state, agent, progress, filename, rd=None,
                         save_pickle=False):

    if rd is None:
        rd = os.path.join(os.path.dirname(filename), 'images')
        
    report = get_agent_report_from_state(state, agent, progress)
    save_report(data_central, report, filename, resources_dir=rd,
                save_pickle=save_pickle) 

@contract(returns=Report,
          agent=AgentInterface, state='*', progress='str')
def get_agent_report_from_state(agent, state, progress):
    rid = ('%s-%s-%s' % (state.id_agent, state.id_robot, progress))

    report = Report(rid)

    stats = ("Num episodes: %s\nNum observations: %s" % 
             (len(state.id_episodes), state.num_observations))
    report.text('learning_statistics', stats)

    agent.publish(report)

    return report

@contract(agent_state='tuple', progress='str')
def get_agentstate_report(agent_state, progress):
    agent, state = agent_state
    return get_agent_report_from_state(agent, state, progress)  


@contract(returns=Report, data_central=DataCentral,
          id_agent='str', id_robot='str', progress='str')
def get_agent_report(data_central, id_agent, id_robot, progress):
    """ 
        The report name is ('%s-%s-%s' % (state.id_agent, state.id_robot, progress)),
        you might want to choose "progress" to not have conflicts.
    """
    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=False)
    return get_agent_report_from_state(agent, state, progress)

    
#    
#    
# def add_robot_info(data_central, report, id_robot):
#    robot = data_central.get_bo_config().robots.instance(id_robot)
#    
#    if isinstance(robot, EquivRobot):
#        add_nuisances_info(robot, report)
#    
#    try: 
#        vsim = get_vsim_from_robot(robot)
#    except:
#        # not a Vehicle
#        return
#    else:
#        add_vehicle_info(vsim, report)
#    
#    
# def add_nuisances_info(robot, report):
#    sec = report.section('nuisances')
#    obs, cmd = robot.get_nuisances()
#
#    sec.data('observations', obs)
#    sec.data('commands', cmd)
#     
#     
# def add_vehicle_info(vsim, report):
#    vsim.new_episode()
#    vsim.compute_observations()
#    sim_state = vsim.to_yaml()
#
#    plot_params = dict(grid=0,
#                       zoom=1.5,
#                       zoom_scale_radius=True,
#                       width=500, height=500,
#                       show_sensor_data=False,
#                       show_sensor_data_compact=True,
#                       bgcolor=None,
#                       show_world=False)
#
#    from vehicles_cairo.write_to_file import vehicles_cairo_display_pdf
#
#    sec = report.section('vehicle')
#    
#    shots = {
#         'body': {},
#         'body_data': dict(show_sensor_data=True,
#                           show_sensor_data_compact=False),
#         'body_data_compact': dict(show_sensor_data=True,
#                           show_sensor_data_compact=True),
#    }
#    
#    for name, options in shots.items():
#        with sec.data_file(name, MIME_PDF) as filename:
#            p = dict(**plot_params)
#            p.update(options)
#            vehicles_cairo_display_pdf(filename, sim_state=sim_state, **p)

