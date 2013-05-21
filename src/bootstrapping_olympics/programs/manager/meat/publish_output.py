from . import load_agent_state, save_report
import os

__all__ = ['publish_once', 'publish_agent_output']

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
    rid = ('%s-%s-%s' % (state.id_agent, state.id_robot, progress))
    from bootstrapping_olympics.extra.reprep import (boot_has_reprep,
                                                     reprep_error)

    if not boot_has_reprep:
        msg = 'Cannot do this task because Reprep not installed: %s'
        msg = msg % reprep_error
        raise Exception(msg)

    from bootstrapping_olympics.extra.reprep import ReprepPublisher

    publisher = ReprepPublisher(rid)
    report = publisher.r

    stats = ("Num episodes: %s\nNum observations: %s" % 
             (len(state.id_episodes), state.num_observations))
    report.text('learning_statistics', stats)

#     try:
#         agent.display(report)
#     except:
    agent.publish(publisher)

    if rd is None:
        rd = os.path.join(os.path.dirname(filename), 'images')
        
    save_report(data_central, report, filename, resources_dir=rd,
                save_pickle=save_pickle) 
    
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

