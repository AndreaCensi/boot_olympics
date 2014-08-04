from .report_utils import save_report
from blocks import check_timed_named
from bootstrapping_olympics import PredictorAgentInterface
from bootstrapping_olympics.programs.manager.meat import load_agent_state
from bootstrapping_olympics.utils import PredictionStats
from contracts.utils import check_isinstance
from numpy.ma.core import allclose
from reprep import Report
import numpy as np
import warnings

__all__ = [
    'task_predict', 
    'predict_report',
]


def task_predict(data_central, id_agent, id_robot,
                 from_start_command=0.5,
                 interval_min=0.10,
                 interval_max=0.15,
                 remove_invalid=True,
                 live_plugins=[]):
    log_index = data_central.get_log_index()
    boot_spec = log_index.get_robot_spec(id_robot)

    agent, state = load_agent_state(data_central, id_agent, id_robot,
                                    reset_state=False,
                                    raise_if_no_state=True)

    predictor = agent.get_predictor()
    predictor.init(boot_spec)
    
    check_isinstance(predictor, PredictorAgentInterface)

    streams = log_index.get_streams_for_robot(id_robot)
    p0 = 0.005
    
    from astatsa.prediction import PredictionStatsSampled

    y_dot_stats = PredictionStats('y_dot', 'y_dot_pred')
    y_dot_stats2 = PredictionStatsSampled(p0, 'y_dot', 'y_dot_pred')
    y_delta_stats2 = PredictionStatsSampled(p0, 'y_delta', 'y_delta_pred')
    
    y_dot_sign_stats = PredictionStats('y_dot_sign', 'y_dot_pred_sign')
    y_dot_sign_stats2 = PredictionStatsSampled(p0, 'y_dot_sign', 'y_dot_pred_sign')
    
    u_stats = PredictionStats('u', 'u_pred')
    
    # Initialize plugins
    live_plugins = [data_central.get_bo_config().live_plugins.instance(x) 
                    for x in live_plugins]
    for plugin in live_plugins:
        plugin.init(dict(data_central=data_central,
                         id_agent=id_agent, id_robot=id_robot))

    
    samples = predict_all_streams(streams=streams,
                                  predictor=predictor,
                                  from_start_command=from_start_command,
                                  interval_min=interval_min,
                                  interval_max=interval_max)

    for sample in samples:
        compute_errors(sample)
        
        # remove invalid data
        y2 = sample['y']
        y1 = sample['y_prev']
        
        w = np.ones(y1.shape)
        if remove_invalid:
            w = np.logical_and(w, y1 > 0)
            w = np.logical_and(w, y1 < 1)
            w = np.logical_and(w, y2 > 0)
            w = np.logical_and(w, y2 < 1)
            w = np.logical_and(w, np.logical_not(np.allclose(sample['y_dot'], 0)))
        
        y_dot_stats.update(sample['y_dot'], sample['y_dot_pred'])
        y_dot_stats2.update(sample['y_dot'], sample['y_dot_pred'], w)
        y_delta_stats2.update(sample['y_delta'], sample['y_delta_pred'], np.ones(w.size))
        
        y_dot_sign_stats.update(sample['y_dot_sign'], sample['y_dot_pred_sign'])
        y_dot_sign_stats2.update(sample['y_dot_sign'], sample['y_dot_pred_sign'], w)
        u_stats.update(sample['commands'], sample['est_u'])
        
        # Update plugins
        for plugin in live_plugins:
            plugin.update(dict(agent=agent, robot=None, obs=sample['seq'][0],
                            predict=sample))
    
    params = dict(from_start_command=from_start_command,
                  interval_min=interval_min, interval_max=interval_max,
                  remove_invalid=remove_invalid)
    statistics = dict(y_dot_stats=y_dot_stats,
                      y_dot_stats2=y_dot_stats2,
                      y_dot_sign_stats=y_dot_sign_stats,
                      y_delta_stats2=y_delta_stats2,
                      y_dot_sign_stats2=y_dot_sign_stats2,
                      u_stats=u_stats,
                      id_state=state.id_state,
                      params=params)
    return statistics


def report_task_predict(statistics):
    r = Report()
    
    r.text('params', str(statistics['params']))
    
    u_stats = statistics['u_stats'] 
    y_dot_stats = statistics['y_dot_stats'] 
    y_dot_sign_stats = statistics['y_dot_sign_stats']

    with r.subsection('y_dot') as sub:
        y_dot_stats.publish(sub)

    with r.subsection('y_dot2') as sub:
        statistics['y_dot_stats2'].publish(sub)

    with r.subsection('y_delta') as sub:
        statistics['y_delta_stats2'].publish(sub)
        
    with r.subsection('y_dot_sign') as sub:
        y_dot_sign_stats.publish(sub)

    with r.subsection('y_dot_sign2') as sub:
        statistics['y_dot_sign_stats2'].publish(sub)

    with r.subsection('u') as sub:
        u_stats.publish(sub)
    
    return r


def predict_report(data_central, id_agent, id_robot, statistics, save_pickle=False):

    u_stats = statistics['u_stats'] 
    y_dot_stats = statistics['y_dot_stats'] 
    y_dot_sign_stats = statistics['y_dot_sign_stats']
    id_state = statistics['id_state']
    
    basename = 'pred-%s-%s' % (id_agent, id_robot)
    
    r = Report(basename)
    
    y_dot_stats.publish(r.section('y_dot'))
    y_dot_sign_stats.publish(r.section('y_dot_sign'))
    u_stats.publish(r.section('u'))
    
    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_res_dir(id_agent=id_agent, id_robot=id_robot,
                                       id_state=id_state, phase='predict')
    filename = ds.get_report_filename(id_agent=id_agent, id_robot=id_robot,
                                       id_state=id_state, phase='predict')
    
    save_report(data_central, r, filename, resources_dir=report_dir,
                save_pickle=save_pickle) 
    

def compute_errors(s):
    ob1 = s['ob1']
    ob2 = s['ob2']
    dt = ob2['timestamp'] - ob1['timestamp']
    assert dt > 0
    s['y'] = ob2['observations']
    s['y_prev'] = ob1['observations']
    s['y_delta'] = (s['y'] - s['y_prev']) 
    s['y_dot'] = (s['y'] - s['y_prev']) / dt
    s['y_dot_sign'] = np.sign(s['y_dot'])


    s['y_pred'] = s['predict_y']
    s['y_delta_pred'] = s['y_pred'] - s['y_prev']
    s['y_dot_pred'] = (s['y_pred'] - s['y_prev']) / dt
    s['y_dot_pred_sign'] = np.sign(s['y_dot_pred'])

    errors = {}

    def compare(a, b, prefix):
        errors['%s_L2' % prefix] = np.linalg.norm(a - b, ord=2)
        errors['%s_L1' % prefix] = np.linalg.norm(a - b, ord=1)

    compare(s['y'], s['y_pred'], 'y')
    compare(s['y_dot'], s['y_dot_pred'], 'y_dot')

    s['errors'] = errors


def predict_all_streams(streams, predictor, interval_min, interval_max,
                        from_start_command,
                        exclude_zero_commands=True, skip_initial=5):
    '''
        Iterates over all observations in the stream.
        
        Calls predictor.
    
    
        yields dict with fields:
             "data": current observations 
             "prev": previous observations
             
             "predict_y"
             "est_u" 
    
    '''
    for stream in streams:
        seqs = get_subs_same_u(stream.read(read_extra=False))
        counter = 0
        for seq in seqs:
            counter += 1
            dtall = seq[-1]['timestamp'] - seq[0]['timestamp']
            
            timestamps = np.array([s['timestamp'] for s in seq])
            
            if dtall <= 0:
                msg = 'Timestamps-t0: %s' % (timestamps - timestamps[0])
                raise Exception(msg)
            commands = seq[0]['commands']
            
            
            nsub = 0

            skip = False
            
            if counter <= skip_initial:
                skip = True
                
            if exclude_zero_commands and np.allclose(commands, 0):
                skip = True

            if not skip:
                for i in range(len(timestamps)):
                    t0 = timestamps[i]
                    # this helps with sync issues (see also below)
                    if t0 <= timestamps[0] + from_start_command:
                        continue 
                    t1a = t0 + interval_min
                    t1b = t0 + interval_max
                    ok = np.logical_and(timestamps > t0,
                                        np.logical_and(timestamps >= t1a,
                                                       timestamps <= t1b))

                    # Here we process the first observation
                    warnings.warn('This only works with memoryless predictors')
                    for i1 in range(0, i + 1):
                        predictor.process_observations(seq[i1])
                    
                    # if not np.any(ok):
                    #    print('no valid matches (%s)' % (timestamps - t0))
                    
                    for j in np.nonzero(ok)[0]:
                        

                        # this helps with sync issues (see also below)
                        if timestamps[j] > timestamps[-1] - from_start_command:
                            continue
                        
                        dt = seq[j]['timestamp'] - seq[i]['timestamp']
#                         dt = float(dt)
                        
#                         print dt
                        # assert interval_min <= dt <= interval_max, (interval_min, dt, interval_max, timestamps)
                         
                        predict_y = predictor.predict_y(dt=dt)
                        
                        warnings.warn('This also is not good')
                        est_u = predictor.estimate_u()
                        
                        nsub += 1
                        yield dict(ob1=seq[i],
                                   ob2=seq[j],
                                   commands=commands,
                                   predict_y=predict_y,
                                   est_u=est_u)
    
            print('len(seq): %4d  delta: %8.2f nsub: %3d u: %s' % (len(seq), dtall, nsub, commands.tolist()))
    
            # Here we process the rest (except the last, which is in common
            # with the next sequence
            for ob in seq[1:-1]:
                predictor.process_observations(ob)
            # last_observations = observations
                        
#         last_observations = None
#         for observations in stream.read(read_extra=False):
#             if observations['counter'] > skip_initial:
#                 predict_y = predictor.predict_y(dt=observations['dt'])

#                 if not isinstance(predict_y, np.ndarray):
#                     msg = 'Want array, got %s' % describe_type(predict_y)
#                     raise Exception(msg)
# 
#                 expected = observations['observations'].shape
#                 found = predict_y.shape
#                 if expected != found:
#                     msg = 'Want shape %s, got %s.' % (expected, found)
#                     raise Exception(msg)


def get_subs_same_u(stream, skip_initial=5):
    raise NotImplementedError('to do')

    current_u = None
    current_episode = None
    seq = []
    counter = 0
    for x in stream:
        counter += 1
        if counter < skip_initial:
            continue

        check_timed_named(x)
        (t, (signal, value)) = x
        if signal == 'id_episode':
            current_episode = value
        elif signal == 'commands':
            commands = value
            
        if current_u is None:
            current_u = commands
            seq = [obs]
        else:
            if allclose(current_u, commands) and (obs['id_episode'] == current_episode):
                seq.append(obs)
            else:
                if (obs['id_episode'] == current_episode):
                    seq.append(obs)
                    
                if len(seq) > 1: 
                    assert seq[-1]['timestamp'] > seq[0]['timestamp']
                    yield seq
                current_u = commands
                current_episode = obs['id_episode']
                seq = [obs]
        

