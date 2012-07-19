from . import load_agent_state, np, save_report
from contracts import describe_type

__all__ = ['task_predict', 'predict_report']


def task_predict(data_central, id_agent, id_robot, live_plugins=[]):
    # TODO FIXME: remove dependency on boot_agents
    from boot_agents.utils import PredictionStats
    from bootstrapping_olympics.extra.reprep import (boot_has_reprep,
                                                     reprep_error)

    if not boot_has_reprep:
        msg = 'Cannot do this task because Reprep not installed: %s'
        msg = msg % reprep_error
        raise Exception(msg)

    agent, state = load_agent_state(data_central, id_agent, id_robot,
                             reset_state=False,
                             raise_if_no_state=True)

    predictor = agent.get_predictor()

    log_index = data_central.get_log_index()
    streams = log_index.get_streams_for_robot(id_robot)
    y_dot_stats = PredictionStats('y_dot', 'y_dot_pred')
    y_dot_sign_stats = PredictionStats('y_dot_sign', 'y_dot_pred_sign')
    
    # Initialize plugins
    live_plugins = [data_central.get_bo_config().live_plugins.instance(x) 
                    for x in live_plugins]
    for plugin in live_plugins:
        plugin.init(dict(data_central=data_central,
                         id_agent=id_agent, id_robot=id_robot))

    for sample in predict_all_streams(streams=streams, predictor=predictor):
        compute_errors(sample)
        y_dot_stats.update(sample['y_dot'], sample['y_dot_pred'])
        y_dot_sign_stats.update(sample['y_dot_sign'], sample['y_dot_pred_sign'])
        
        # Update plugins
        for plugin in live_plugins:
            plugin.update(dict(agent=agent, robot=None, obs=sample['data'],
                            predict=sample))
    
    statistics = dict(y_dot_stats=y_dot_stats,
                      y_dot_sign_stats=y_dot_sign_stats,
                      id_state=state.id_state)
    return statistics


def predict_report(data_central, id_agent, id_robot, statistics, save_pickle=False):
    from bootstrapping_olympics.extra.reprep import ReprepPublisher

    y_dot_stats = statistics['y_dot_stats'] 
    y_dot_sign_stats = statistics['y_dot_sign_stats']
    id_state = statistics['id_state']
    
    basename = 'pred-%s-%s' % (id_agent, id_robot)
    from reprep import Report
    r = Report(basename)
    publisher = ReprepPublisher(report=r)
    y_dot_stats.publish(publisher.section('y_dot'))
    y_dot_sign_stats.publish(publisher.section('y_dot_sign'))

    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_res_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=id_state,
                                       phase='predict')
    filename = ds.get_report_filename(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=id_state,
                                       phase='predict')
    
    save_report(data_central, r, filename, resources_dir=report_dir,
                save_pickle=save_pickle) 
    

def compute_errors(s):
    data = s['data']
    prev = s['prev']
    dt = data['dt']
    s['y'] = data['observations']
    s['y_prev'] = prev['observations']
    s['y_dot'] = (s['y'] - s['y_prev']) / dt
    s['y_dot_sign'] = np.sign(s['y_dot'])

    s['y_pred'] = s['predict_y']
    s['y_dot_pred'] = (s['y_pred'] - s['y_prev']) / dt
    s['y_dot_pred_sign'] = np.sign(s['y_dot_pred'])

    errors = {}

    def compare(a, b, prefix):
        errors['%s_L2' % prefix] = np.linalg.norm(a - b, ord=2)
        errors['%s_L1' % prefix] = np.linalg.norm(a - b, ord=1)

    compare(s['y'], s['y_pred'], 'y')
    compare(s['y_dot'], s['y_dot_pred'], 'y_dot')

    s['errors'] = errors


def predict_all_streams(streams, predictor, skip_initial=5):
    ''' yields dict with fields "data", "predict_y" '''
    for stream in streams:
        last_observations = None
        for observations in stream.read(read_extra=False):
            if observations['counter'] > skip_initial:
                predict_y = predictor.predict_y(dt=observations['dt'])

                if not isinstance(predict_y, np.ndarray):
                    msg = 'Want array, got %s' % describe_type(predict_y)
                    raise Exception(msg)

                expected = observations['observations'].shape
                found = predict_y.shape
                if expected != found:
                    msg = 'Want shape %s, got %s.' % (expected, found)
                    raise Exception(msg)

                yield dict(prev=last_observations,
                           data=observations,
                           predict_y=predict_y)

            predictor.process_observations(observations)
            last_observations = observations

