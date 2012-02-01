from . import logger
from .. import all_nuisances, for_all_robots
from bootstrapping_olympics import (BootOlympicsConfig, StreamSpec,
    UnsupportedSpec)
from bootstrapping_olympics.utils import assert_allclose


@for_all_robots
def check_nuisances(id_robot, robot):
    for id_nuisance in all_nuisances():
        #print('Testing %s / %s ' % (id_robot, id_nuisance))
        nuisance = BootOlympicsConfig.specs['nuisances'].instance(id_nuisance)
        boot_spec = robot.get_spec()
        check_conversions(boot_spec.get_commands(), nuisance)
        check_conversions(boot_spec.get_observations(), nuisance)


def check_conversions(stream_spec1, nuisance):
    #print('Checking %s / %s ' % (stream_spec1, nuisance))
    nuisance_inv = nuisance.inverse()
    try:
        try:
            stream_spec2 = nuisance.transform_spec(stream_spec1)
        except UnsupportedSpec as e:
            logger.info('Skipping %s/%s because incompatible: %s' %
                        (stream_spec1, nuisance, e))
            return
        stream_spec1b = nuisance_inv.transform_spec(stream_spec2)
        StreamSpec.check_same_spec(stream_spec1, stream_spec1b)

        value1 = stream_spec1.get_random_value()
        stream_spec1.check_valid_value(value1)

        value2 = nuisance.transform_value(value1)
        stream_spec2.check_valid_value(value2)

        value1b = nuisance_inv.transform_value(value2)
        stream_spec1.check_valid_value(value1b)

        # TODO: if exact
        assert_allclose(value1, value1b)

    except:
        logger.error('Error while testing:')
        logger.error(' stream_spec:  %s ' % stream_spec1)
        logger.error(' nuisance:     %s' % nuisance)
        logger.error(' nuisance_inv: %s' % nuisance_inv)
        raise

