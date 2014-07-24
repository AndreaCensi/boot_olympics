from bootstrapping_olympics.programs.manager.meat.report_robot import report_robot_create

__all__ = [ 'jobs_task_robot_report']

def jobs_task_robot_report(context, id_robot):
    r = context.comp_config(report_robot_create, id_robot)
    context.add_report(r, 'robot', id_robot=id_robot)
