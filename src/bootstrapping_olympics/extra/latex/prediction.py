import itertools
from reprep.constants import MIME_PNG
from bootstrapping_olympics.extra.latex.load import load_report_phase
from latex_gen.frags import latex_fragment
import sys
from bootstrapping_olympics.extra.latex.vehicle import create_robot_figure


def read_reports(id_set, robots, agents, phase):
    reports = {}
    for robot, agent in itertools.product(robots, agents):
        reports[(robot, agent)] = load_report_phase(id_set, agent, robot, phase)
    return reports


def prediction_report(id_set, robots, agents, draft=False, tsize='3cm'):
    reports = read_reports(id_set, robots, agents, 'predict')
    
    def write_graphics_data(el, data_node, gid):
        # Add here low resolution
        # In draft mode, do not output pngs
        use_draft = draft and MIME_PNG == data_node.mime
        
        if not use_draft:
            el.graphics_data(data_node.raw_data, data_node.mime,
                                  width=tsize, gid=gid)
        else:
            el.rule(tsize, tsize, color='gray')
    
    def boxed(el, data_node, gid):
        with el.tightbox() as box:
            write_graphics_data(box, data_node, gid)

    graphics_path = 'snippets/figures'
    with latex_fragment(sys.stdout, graphics_path=graphics_path) as frag:
        caption = 'Report for robots %r, agents %r. ' % (robots, agents)
        
        alignment = ['r'] + ['c'] * len(agents)
        with frag.tabular(alignment=alignment) as tabular:
            
            with tabular.row() as header:
                header.cell_tex() 
                for agent in agents:
                    header.cell_tex(agent)
    
            for robot in robots:
                with tabular.row() as row:
                    
                    with row.cell() as cell:
                        with cell.minipage('3cm', align='b') as mp:
                            mp.tex('\\centering')
                            create_robot_figure(mp, id_set, robot)
                            mp.linebreak()
                            mp.tex(robot)
                        
                    for agent in agents:
                        r = reports[(robot, agent)]
                        sys.stderr.write(r.format_tree())
                        image = r['y_dot/figure1/correlation']
                        with row.cell() as cell:
                            gid = '%s-%s-predict-y_dot-corr' % (robot, agent)
                            write_graphics_data(cell, image, gid=gid)
            
