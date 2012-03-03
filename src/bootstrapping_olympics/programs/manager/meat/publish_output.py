from . import load_agent_state, logger
from bootstrapping_olympics.utils import isodate
import os


def publish_once(data_central, id_agent, id_robot,
                 phase='learn', progress='all'): # TODO: 'learn' in constants
    # XXX: progres is not used so far
    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=False)

    ds = data_central.get_dir_structure()
    filename = ds.get_report_filename(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=state.id_state,
                                       phase=phase)

    res_dir = ds.get_report_res_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=state.id_state,
                                       phase=phase)

    publish_agent_output(state, agent, progress, filename=filename, rd=res_dir)
    ds.file_is_done(filename, desc="publish_once(%s,%s,%s,%s)" %
                    (id_agent, id_robot, phase, progress))


def publish_agent_output(state, agent, progress, filename, rd=None):
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

    report.text('report_date', isodate())

    agent.publish(publisher)

    logger.info('Writing to %r.' % filename)

    if rd is None:
        rd = os.path.join(os.path.dirname(filename), 'images')
    report.to_html(filename, resources_dir=rd)

