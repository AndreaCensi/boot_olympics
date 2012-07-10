from . import np, logger
from reprep.plot_utils.styles import (style_ieee_fullcol_xy,
    style_ieee_halfcol_xy)
from geometry.poses import translation_from_SE2


def servo_stats_report(data_central, id_agent, id_robot, summaries,
                       phase='servo_stats'):
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

        trans = [translation_from_SE2(x) for x in s['poses']]
        s['x'] = np.abs([t[0] for t in trans])
        s['y'] = np.abs([t[1] for t in trans])
        s['dist_x'] = np.abs(s['x'])
        s['dist_y'] = np.abs(s['y'])

    basename = 'servo_analysis-%s-%s-%s' % (id_agent, id_robot, phase)
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

    with f.plot('xy') as pylab:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pylab.plot(s['x'], s['y'], s['color'] + '-')
        pylab.xlabel('x')
        pylab.ylabel('y')

    with f.plot('dist_th') as pylab:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pylab.plot(s['dist_th'], s['color'] + '-')

    with f.plot('dist_th_deg') as pylab:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pylab.plot(np.rad2deg(s['dist_th']), s['color'] + '-')

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
    
    with f.plot('dist_y') as pylab:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pylab.plot(s['dist_y'], s['color'] + '-')
    with f.plot('dist_x') as pylab:
        style_ieee_fullcol_xy(pylab)
        for s in summaries:
            pylab.plot(s['dist_x'], s['color'] + '-')

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
            if s['converged']: 
                continue

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
    filename = ds.get_report_filename(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state='servo_stats',
                                       phase=phase)
    res_rd = ds.get_report_res_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state='servo_stats',
                                       phase=phase)
#    filename = os.path.join(report_dir, 'servo_stats_report.html')
    logger.info('Writing output to %r.' % filename)
    r.to_html(filename, resources_dir=res_rd)
