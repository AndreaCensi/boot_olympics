from bootstrapping_olympics.library.robots import EquivRobot
from reprep import MIME_PDF, MIME_SVG, Report
from .report_utils import save_report
from .servo.utils import get_vsim_from_robot
from bootstrapping_olympics.configuration.master import get_conftools_robots


# def publish_report_robot(data_central, id_robot, save_pickle=False):
#     ds = data_central.get_dir_structure()
#     filename, resources_dir = ds.get_report_robot_filename_rd(id_robot)
#     save_report(data_central, report, filename, resources_dir, save_pickle=save_pickle)

def report_robot_create(id_robot):
    report = Report()
    add_robot_info(report, id_robot)
    return report


def add_robot_info(report, id_robot):
    robot = get_conftools_robots().instance(id_robot)
    
    if isinstance(robot, EquivRobot):
        add_nuisances_info(robot, report)
    try: 
        vsim = get_vsim_from_robot(robot)
    except:
        # not a Vehicle
        return
    else:
        add_vehicle_info(vsim, report)
    
    
def add_nuisances_info(robot, report):
    sec = report.section('nuisances')
    obs, cmd = robot.get_nuisances()

    sec.data('observations', obs)
    sec.data('commands', cmd)
     
     
def add_vehicle_info(vsim, report):
    
    sec_vehicle = report.section('vehicle')
    sec_environment = report.section('environment')

    sec_vehicle.data('id_vehicle', vsim.id_vehicle)
    sec_vehicle.data('vehicle_spec', vsim.vehicle_spec)
    sec_environment.data('id_world', vsim.id_world)
    sec_environment.data('world_spec', vsim.world_spec)
        
    vsim.new_episode()
    vsim.compute_observations()
    sim_state = vsim.to_yaml()

    plot_params = dict(grid=0,
                       zoom=1.5,
                       zoom_scale_radius=True,
                       width=500, height=500,
                       show_sensor_data=False,
                       show_sensor_data_compact=True,
                       bgcolor=None,
                       show_world=False)

    from vehicles_cairo.write_to_file import vehicles_cairo_display_pdf
    from vehicles_cairo.write_to_file import vehicles_cairo_display_svg
    
    shots = {
         'body': {},
         'body_data': dict(show_sensor_data=True,
                           show_sensor_data_compact=False),
         'body_data_compact': dict(show_sensor_data=True,
                           show_sensor_data_compact=True),
    }
    

    for name, options in shots.items():
        f = sec_vehicle.figure('f%s' % name)

        with sec_vehicle.data_file(name, MIME_PDF) as filename:
            p = dict(**plot_params)
            p.update(options)
            vehicles_cairo_display_pdf(filename, sim_state=sim_state, **p)

        f.sub(sec_vehicle.last())

        with sec_vehicle.data_file(name + '_svg', MIME_SVG) as filename:
            vehicles_cairo_display_svg(filename, sim_state=sim_state, **p)

        f.sub(sec_vehicle.last())
