from bootstrapping_olympics.configuration.master import BootOlympicsConfig
from bootstrapping_olympics.programs.manager.meat.servonav.find_path import (
    get_grid)
from vehicles import VehiclesConfig
from vehicles_cairo import (cairo_ref_frame, cairo_rototranslate,
    vehicles_cairo_display_png)
from geometry import SE2_from_SE3
from pprint import pprint
from bootstrapping_olympics.utils import yaml_dump


def main():
    # Instance robot object
    id_robot = 'r_cam_A'
    filename = 'test.png'
    resolution = 0.5

    config = BootOlympicsConfig
    vconfig = VehiclesConfig
    config.load('/Users/andrea/scm/boot11_env/src/vehicles/src/vehicles/configs')
    config.load('/Users/andrea/scm/boot11_env/src/bvapps/bo_app1/config')
    vconfig.load() # Default
    vconfig.load('/Users/andrea/scm/boot11_env/src/bvapps/bo_app1/config')

    robot = config.robots.instance(id_robot) #@UndefinedVariable
    robot.new_episode()

    locations = get_grid(robot=robot, debug=True,
                        world=robot.world,
                        vehicle=robot.vehicle, resolution=resolution)
    poses = [f['pose'] for f in locations]

#    poses = elastic(poses, alpha=0.1, num_iterations=20)

    print('Converting to yaml...')
    robot.vehicle.set_pose(poses[0])
    state = robot.to_yaml()

    pprint(state)
    print(yaml_dump(state))

    def extra_draw_world(cr):
        for pose in poses:
            with cairo_rototranslate(cr, SE2_from_SE3(pose)):
                cairo_ref_frame(cr, l=0.5,
                            x_color=[0, 0, 0], y_color=[0, 0, 0])

    plotting_params = {}
    plotting_params['extra_draw_world'] = extra_draw_world

    print('Writing to: %r' % filename)
    vehicles_cairo_display_png(filename, width=800, height=800, sim_state=state,
                               **plotting_params)

    print('... done')


if __name__ == '__main__':
    main()
