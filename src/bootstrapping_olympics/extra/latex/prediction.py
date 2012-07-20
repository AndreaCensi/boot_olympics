from . import bvid, create_robot_figure, get_resources_dir
from latex_gen import latex_fragment
import sys
from bootstrapping_olympics.extra.latex.utils import get_sensel_pgftable
from contracts import contract
from reprep.constants import MIME_PLAIN
from bootstrapping_olympics.extra.latex.load import load_report_phase


@contract(R='array[N]')
def display_correlation(fig, R, gid, width):
    table = get_sensel_pgftable(R, 'corr', 'Correlation for %s' % gid)
    fig.save_graphics_data(table, MIME_PLAIN, gid)
    fig.tex('\\corrFigure{%s}{%s}' % (gid, width))
    

def get_predict_corr(id_set, agent, robot):
    report_predict = load_report_phase(id_set, agent, robot, 'predict')
    #y_dot = report_predict['y_dot']
    #sys.stderr.write(y_dot.format_tree())
    R = report_predict['y_dot/R'].raw_data 
    # R = np.cos(np.linspace(0, np.pi * 2, 240))
    return R


def fig_predict_corr(frag, id_set, id_agent, id_robot, width='3cm'):
    prefix = '%s-%s-%s' % (id_set, id_robot, id_agent)
    gid = prefix + '-pred-corr'
    R = get_predict_corr(id_set, id_agent, id_robot)
    display_correlation(frag, R, gid, width)


def prediction_report(id_set, robots, agents, width='3cm'):
    with latex_fragment(sys.stdout, graphics_path=get_resources_dir()) as frag:
        alignment = ['r'] + ['c'] * len(agents)
        with frag.tabular(alignment=alignment) as tabular:
            
            with tabular.row() as header:
                header.cell_tex() 
                for agent in agents:
                    header.cell_tex(bvid(agent))
    
            for robot in robots:
                with tabular.row() as row:
                    
                    with row.cell() as cell:
                        with cell.minipage(width, align='b') as mp:
                            mp.tex('\\centering')
                            create_robot_figure(mp, id_set, robot)
                            mp.linebreak()
                            mp.tex(robot)
                        
                    for agent in agents:
                        with row.cell() as cell:
                            fig_predict_corr(cell, id_set, agent,
                                             robot, width=width)
            
