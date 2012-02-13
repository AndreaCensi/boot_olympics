from . import logger
from bootstrapping_olympics import (StreamSpec, UnsupportedSpec,
    NuisanceNotInvertible)
from bootstrapping_olympics.unittests.tests_generation import (
    for_all_robot_nuisance_pairs)
from bootstrapping_olympics.utils import assert_allclose, indent


@for_all_robot_nuisance_pairs
def check_nuisances_cmd(id_robot, robot, id_nuisance, nuisance):
    check_conversions(robot.get_spec().get_commands(), nuisance)


@for_all_robot_nuisance_pairs
def check_nuisances_obs(id_robot, robot, id_nuisance, nuisance):
    check_conversions(robot.get_spec().get_observations(), nuisance)


def check_conversions(stream_spec1, nuisance):
    #print('Checking %s / %s ' % (stream_spec1, nuisance))
    nuisance_inv = None
    try:
        try:
            stream_spec2 = nuisance.transform_spec(stream_spec1)
        except UnsupportedSpec as e:
            logger.info('Skipping %s/%s because incompatible: %s' %
                        (stream_spec1, nuisance, e))
            return

        value1 = stream_spec1.get_random_value()
        stream_spec1.check_valid_value(value1)

        value2 = nuisance.transform_value(value1)
        stream_spec2.check_valid_value(value2)

        try:
            nuisance_inv = nuisance.inverse()
        except NuisanceNotInvertible as e:
            logger.info('Skipping some tests %s/%s because not invertible:'
                        ' %s' % (stream_spec1, nuisance, e))
            return

        try:
            stream_spec1b = nuisance_inv.transform_spec(stream_spec2)
        except UnsupportedSpec as e:
            msg = ('The inverse of the nuisance does not seem to be able '
                   'to handle the result:\n%s\n\n'
                   '  stream_spec1: %s\n'
                   '      nuisance: %s\n'
                   '  stream_spec2: %s\n'
                   '  nuisance_inv: %s\n' %
                   (indent(str(e), '>'),
                    stream_spec1.to_yaml(), nuisance,
                    stream_spec2.to_yaml(), nuisance_inv))
            raise ValueError(msg)

        try:
            StreamSpec.check_same_spec(stream_spec1, stream_spec1b)
        except Exception as e:
            msg = ('The inverse of the nuisance does not recreate the '
                   'initial spec:\n%s\n\n'
                   '  stream_spec1: %s\n'
                   '      nuisance: %s\n'
                   '  stream_spec2: %s\n'
                   '  nuisance_inv: %s\n'
                   ' stream_spec1b: %s\n' %
                   (indent(str(e), '>'),
                    stream_spec1.to_yaml(), nuisance,
                    stream_spec2.to_yaml(), nuisance_inv,
                    stream_spec1b.to_yaml()))
            raise ValueError(msg)

        value1b = nuisance_inv.transform_value(value2)
        stream_spec1.check_valid_value(value1b)

        # TODO: if exact
        assert_allclose(value1, value1b, rtol=1e-5)

    except:
        logger.error('Error while testing:')
        logger.error(' stream_spec:  %s ' % stream_spec1.to_yaml())
        logger.error(' nuisance:     %s' % nuisance)
        logger.error(' nuisance_inv: %s' % nuisance_inv)
        raise

