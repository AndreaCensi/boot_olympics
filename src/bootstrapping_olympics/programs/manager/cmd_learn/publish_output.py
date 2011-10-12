from .. import logger
from ....display import ReprepPublisher 
import os
from ....utils import isodate

def publish_once(data_central, id_agent, id_robot):
    from .cmd_learn import load_agent_state
    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=False)
    
    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=state.id_state)
    publish_agent_output(state, agent, report_dir)

    

def publish_agent_output(state, agent, pd):
    rid = ('%s-%s-%s-%07d' % (state.id_agent, state.id_robot,
                            state.id_state, state.num_observations))
    publisher = ReprepPublisher(rid)
    report = publisher.r
    
    stats = ("Num episodes: %s\nNum observations: %s" % 
             (len(state.id_episodes), state.num_observations))
    report.text('learning statistics', stats)
    
    report.text('report_date', isodate())

    agent.publish(publisher)
    filename = os.path.join(pd, '%s.html' % rid)
#    global once
#    if not once:
#        once = True
    logger.info('Writing to %r.' % filename)
#    else:
#        #logger.debug('Writing to [...]/%s .' % os.path.basename(filename))
#        pass
    rd = os.path.join(pd, 'images')
    report.to_html(filename, resources_dir=rd)
    
    last = os.path.join(pd, 'last.html')
    if os.path.exists(last):
        os.unlink(last)
    os.link(filename, last)
    
