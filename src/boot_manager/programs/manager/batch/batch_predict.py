import warnings

__all__ = ['jobs_tasks_predict']


def jobs_tasks_predict(context, data_central, id_agent, id_robot,
                       agent_has_learned, 
                       live_plugins=[],
                      save_pickle=True):
    warnings.warn('FIXME: here we are using *all* streams') 

    from boot_manager.meat import task_predict, predict_report
    
    statistics = context.comp_config(task_predict, data_central=data_central,
         id_agent=id_agent, id_robot=id_robot, live_plugins=live_plugins,
         job_id='predict-%s-%s' % (id_robot, id_agent),
         extra_dep=agent_has_learned)
    
    context.comp_config(predict_report, data_central=data_central,
                         id_agent=id_agent, id_robot=id_robot,
                         statistics=statistics, save_pickle=save_pickle,
         job_id='report-predict-%s-%s' % (id_robot, id_agent))