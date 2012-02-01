from . import np, logger
import os


def servo_stats_report(data_central, id_agent, id_robot, summaries):
    from reprep import Report
    from reprep.plot_utils import x_axis_balanced


    if not summaries:
        raise Exception('Empty summaries')

    def extract(key):
        return np.array([s[key] for s in summaries])

    initial_distance = extract('initial_distance')
    initial_rotation = extract('initial_rotation')

    basename = 'servo_analysis-%s-%s' % (id_agent, id_robot)
    r = Report(basename)

    f = r.figure(cols=1)
    with f.plot('image_L2_error') as pylab:
        for summary in summaries:
            errors = summary['errors']
            pylab.plot(errors)

    with f.plot('dist_xy') as pylab:
        for summary in summaries:
            pylab.plot(summary['dist_xy'])

    with f.plot('dist_th') as pylab:
        for summary in summaries:
            pylab.plot(summary['dist_th'])

    with f.plot('dist_xy_th') as pl:
        for summary in summaries:
            pl.plot(summary['dist_xy'], summary['dist_th'])
        pl.xlabel('dist x-y')
        pl.ylabel('dist th')


    with f.plot('dist_xy_th_log') as pl:
        for summary in summaries:
            dist_xy = summary['dist_xy']
            dist_th = summary['dist_th']
            print dist_xy
            print dist_th
            converged = (dist_xy[0] > dist_xy[-1]) and (dist_th[0] > dist_th[-1])
            if converged:
                color = 'r-'
            else:
                color = 'b-'
            pl.semilogx(summary['dist_xy'], summary['dist_th'], color)
        pl.xlabel('dist x-y')
        pl.ylabel('dist th')


    with f.plot('initial_rotation') as pylab:
        pylab.hist(np.rad2deg(initial_rotation))
        x_axis_balanced(pylab)

    with f.plot('initial_distance') as pylab:
        pylab.hist(initial_distance)

    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state='servo_stats',
                                       phase='servo_stats')
    filename = os.path.join(report_dir, 'servo_stats_report.html')
    logger.info('Writing output to %r.' % filename)
    r.to_html(filename, resources_dir=os.path.join(report_dir, 'images'))
