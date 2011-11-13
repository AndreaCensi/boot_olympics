from . import load_agent_state, logger, np
from ....display import ReprepPublisher
from contracts import describe_type
from reprep import Report # TODO: be safe
import os


__all__ = ['task_predict']


def task_predict(data_central, id_agent, id_robot,
                 interval_print=None):
    ''' Returns the list of the episodes IDs simulated. ''' 
    # Instance agent object    
    from boot_agents.utils import PredictionStats # TODO: remove dependency on boot_agents

    agent, state = load_agent_state(data_central, id_agent, id_robot,
                             reset_state=False,
                             raise_if_no_state=True)
     
    predictor = agent.get_predictor()

    log_index = data_central.get_log_index()
    streams = log_index.get_streams_for_robot(id_robot)
    y_dot_stats = PredictionStats()
    y_dot_sign_stats = PredictionStats()
    
    for sample in predict_all_streams(streams=streams, predictor=predictor):
        compute_errors(sample)
        y_dot_stats.update(sample['y_dot'], sample['y_dot_pred'])
        y_dot_sign_stats.update(sample['y_dot_sign'], sample['y_dot_pred_sign'])
        #print('sample: %s' % (sample['errors']))
        
    basename = 'pred-%s-%s' % (id_agent, id_robot)
    r = Report(basename)
    publisher = ReprepPublisher(report=r)
    
    y_dot_stats.publish(publisher.section('y_dot'))
    y_dot_sign_stats.publish(publisher.section('y_dot_sign'))
    
    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=state.id_state,
                                       phase='predict')
    filename = os.path.join(report_dir, '%s.html' % basename)
    logger.info('Writing output to %r.' % filename)
    r.to_html(filename)
        
        
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
    s['y_dot_pred_sign'] = np.sign(s['y_dot_sign'])
    
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
        
