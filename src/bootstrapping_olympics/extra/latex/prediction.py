from . import create_robot_figure, fig_predict_corr, get_resources_dir
from latex_gen import latex_fragment
import sys
from . import bvid


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
            
