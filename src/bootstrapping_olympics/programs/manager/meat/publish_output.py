from .. import logger
from ....utils import isodate, safe_symlink
from . import load_agent_state
import os

def publish_once(data_central, id_agent, id_robot, phase='learn', progress='all'):
        
    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=False)
    
    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=state.id_state,
                                       phase=phase)
    publish_agent_output(state, agent, report_dir, basename=progress)

    

def publish_agent_output(state, agent, pd, basename):
    rid = ('%s-%s-%07d' % (state.id_agent, state.id_robot,
                               state.num_observations))
    from ....display import ReprepPublisher

    publisher = ReprepPublisher(rid)
    report = publisher.r
    
    stats = ("Num episodes: %s\nNum observations: %s" % 
             (len(state.id_episodes), state.num_observations))
    report.text('learning statistics', stats)
    
    report.text('report_date', isodate())

    agent.publish(publisher)
    filename = os.path.join(pd, '%s.html' % basename)

    logger.info('Writing to %r.' % filename)

    rd = os.path.join(pd, 'images')
    report.to_html(filename, resources_dir=rd)
    
    last = os.path.join(pd, 'last.html')
    safe_symlink(filename, last)

