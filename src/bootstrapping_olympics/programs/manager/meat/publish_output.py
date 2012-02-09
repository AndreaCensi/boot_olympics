from . import load_agent_state, logger
from bootstrapping_olympics.utils import isodate, safe_symlink
import os


def publish_once(data_central, id_agent, id_robot,
                 phase='learn', progress='all'): # TODO: 'learn' in constants

    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=False)

    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=state.id_state,
                                       phase=phase)

    filename = os.path.join(report_dir, '%s.html' % progress)
    publish_agent_output(state, agent, filename=filename)
    ds.file_is_done(filename, desc="publish_once(%s,%s,%s,%s)" %
                    (id_agent, id_robot, phase, progress))


def publish_agent_output(state, agent, filename, rd=None):
    rid = ('%s-%s-%07d' % (state.id_agent, state.id_robot,
                               state.num_observations))
    from ....display import ReprepPublisher

    publisher = ReprepPublisher(rid)
    report = publisher.r

    stats = ("Num episodes: %s\nNum observations: %s" %
             (len(state.id_episodes), state.num_observations))
    report.text('learning_statistics', stats)

    report.text('report_date', isodate())

    agent.publish(publisher)

    logger.info('Writing to %r.' % filename)

    if rd is None:
        rd = os.path.join(os.path.dirname(filename), 'images')
    report.to_html(filename, resources_dir=rd)

#    last = os.path.join(pd, 'last.html')
#    safe_symlink(filename, last)


