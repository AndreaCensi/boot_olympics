from . import load_report_robot
from latex_gen import latex_fragment
import sys


def bvrob(id_robot):
    print('Robot %s' % id_robot)


def bvvehicleimage(id_set, id_robot, width_em=3.5):
    graphics_path = 'snippets/figures' # XXX
    with latex_fragment(sys.stdout, graphics_path=graphics_path) as frag:
        create_robot_figure(frag, id_set, id_robot, width_em)


def create_robot_figure(frag, id_set, id_robot, width_em=3.5):
    report = load_report_robot(id_set, id_robot)
    
    id_world = report['environment/id_world'].raw_data
    id_vehicle = report['vehicle/id_vehicle'].raw_data
    
    has_nuisances = report.has_child('nuisances')
    
    if has_nuisances:
        obs_nuisances = report['nuisances/observations'].raw_data
        for obs_id in obs_nuisances:
            frag.tex('$\\text{%s} \\cdot$' % obs_id)

    #frag.hspace('-5mm')
    # TODO: check vehicle
    vehicle = report['vehicle/body_data_compact']
    
    raiselen = '%.2fem' % (-width_em / 2.0 + 0.5) # 0.5 align middle
    width = '%.2fem' % width_em
    frag.tex('\\raisebox{%s}{%%\n' % raiselen)
    frag.graphics_data(vehicle.raw_data, vehicle.mime, width=width,
                gid='%s-%s-body_data_compact' % (id_set, id_robot))
    frag.tex('}%\n')
    #frag.hspace('-5mm')

    if has_nuisances:
        cmd_nuisances = report['nuisances/commands'].raw_data
        for cmd_id in cmd_nuisances:
            frag.tex('$ \\cdot \\text{%s}$' % cmd_id)
    
    frag.tex('$ \\leftrightarrow \\text{%s} $' % id_world)
    
    
