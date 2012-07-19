

def servo_stats_L2(id_set, agent, robot, width='3cm'):
    prefix = '%s-%s-%s' % (id_set, robot, agent)
    report = load_report_phase(id_set=id_set,
                               agent=agent, robot=robot, phase='servo_stats')
