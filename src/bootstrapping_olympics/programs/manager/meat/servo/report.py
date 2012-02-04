from . import np, logger
import os
from reprep.plot_utils.styles import style_ieee_fullcol_xy, \
    style_ieee_halfcol_xy


def servo_stats_report(data_central, id_agent, id_robot, summaries):
    from reprep import Report
    from reprep.plot_utils import x_axis_balanced

    if not summaries:
        raise Exception('Empty summaries')

    def extract(key):
        return np.array([s[key] for s in summaries])

    initial_distance = extract('initial_distance')
    initial_rotation = extract('initial_rotation')

    dist_xy_converged = 0.25
    dist_th_converged = np.deg2rad(5)

    for s in summaries:
        dist_xy = s['dist_xy']
        dist_th = s['dist_th']
        #converged = (dist_xy[0] > dist_xy[-1]) and (dist_th[0] > dist_th[-1])

        converged = ((dist_xy[-1] < dist_xy_converged) and
                     (dist_th[-1] < dist_th_converged))
        s['converged'] = converged
        s['dist_th_deg'] = np.rad2deg(s['dist_th'])
        s['color'] = 'b' if converged else 'r'

    basename = 'servo_analysis-%s-%s' % (id_agent, id_robot)
    r = Report(basename)

    f = r.figure(cols=3)

    with f.plot('image_L2_error') as pylab:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            errors = s['errors']
            pylab.plot(errors, s['color'])

    with f.plot('dist_xy') as pylab:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pylab.plot(s['dist_xy'], s['color'] + '-')

    with f.plot('dist_th') as pylab:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pylab.plot(s['dist_th'], s['color'] + '-')

    with f.plot('dist_xy_th') as pl:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pl.plot(s['dist_xy'], s['dist_th_deg'], s['color'] + '-')
        pl.xlabel('dist x-y')
        pl.ylabel('dist th (deg)')

    with f.plot('dist_xy_th_log') as pl:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pl.semilogx(s['dist_xy'],
                        s['dist_th_deg'], s['color'] + '.')
        pl.xlabel('dist x-y')
        pl.ylabel('dist th (deg)')

    mark_start = 's'
    mark_end = 'o'

    with f.plot('dist_xy_th_start',
                caption="Initial error (blue: converged)"
                ) as pl:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pl.plot([s['dist_xy'][0], s['dist_xy'][-1]],
                     [s['dist_th_deg'][0],
                       s['dist_th_deg'][-1]], s['color'] + '-')
            pl.plot(s['dist_xy'][0], s['dist_th_deg'][0],
                    s['color'] + mark_start)
            pl.plot(s['dist_xy'][-1], s['dist_th_deg'][-1],
                    s['color'] + mark_end)

        pl.xlabel('dist x-y')
        pl.ylabel('dist th (deg)')

    with f.plot('dist_xy_th_start2',
                caption="Trajectories. If converged, plot square at beginning"
                " and cross at end. If not converged, plot trajectory (red)."
                ) as pl:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            if  s['converged']: continue

            pl.plot([s['dist_xy'][0], s['dist_xy'][-1]],
                     [s['dist_th_deg'][0], s['dist_th_deg'][-1]], 'r-')

        for s in summaries:
            pl.plot(s['dist_xy'][0], s['dist_th_deg'][0], s['color'] + mark_start)
            pl.plot(s['dist_xy'][-1], s['dist_th_deg'][-1], s['color'] + mark_end)

        pl.xlabel('dist x-y')
        pl.ylabel('dist th (deg)')


    with f.plot('initial_rotation') as pylab:
        style_ieee_halfcol_xy(pylab)
        pylab.hist(np.rad2deg(initial_rotation))
        x_axis_balanced(pylab)
        pylab.xlabel('Initial rotation (deg)')

    with f.plot('initial_distance') as pylab:
        style_ieee_halfcol_xy(pylab)
        pylab.hist(initial_distance)
        pylab.xlabel('Initial distance (m)')

    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state='servo_stats',
                                       phase='servo_stats')
    filename = os.path.join(report_dir, 'servo_stats_report.html')
    logger.info('Writing output to %r.' % filename)
    r.to_html(filename, resources_dir=os.path.join(report_dir, 'images'))
