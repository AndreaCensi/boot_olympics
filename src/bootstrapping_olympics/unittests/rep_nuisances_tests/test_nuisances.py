from bootstrapping_olympics import (NuisanceNotInvertible, 
    NuisanceNotLeftInvertible, RepresentationNuisance, StreamSpec, 
    UnsupportedSpec, logger)
from bootstrapping_olympics.unittests import for_all_robot_nuisance_pairs
from bootstrapping_olympics.utils import assert_allclose, indent
from comptests import Skipped
from contracts import describe_value



# @for_all_nuisances
# def check_instance(id_ob, ob):
#     pass


@for_all_robot_nuisance_pairs
def check_nuisances_cmd(id_robot, robot, id_nuisance, nuisance):  # @UnusedVariable
    check_inverse(robot.get_spec().get_commands(), nuisance)


@for_all_robot_nuisance_pairs
def check_nuisances_obs(id_robot, robot, id_nuisance, nuisance):  # @UnusedVariable
    check_inverse(robot.get_spec().get_observations(), nuisance)


@for_all_robot_nuisance_pairs
def check_nuisances_cmd_left(id_robot, robot, id_nuisance, nuisance):  # @UnusedVariable
    check_left_inverse(robot.get_spec().get_commands(), nuisance)

@for_all_robot_nuisance_pairs
def check_nuisances_obs_left(id_robot, robot, id_nuisance, nuisance):  # @UnusedVariable
    check_left_inverse(robot.get_spec().get_observations(), nuisance)


@for_all_robot_nuisance_pairs
def check_nuisances_cmd_left_approx(id_robot, robot, id_nuisance, nuisance):  # @UnusedVariable
    check_left_inverse_approx(robot.get_spec().get_commands(), nuisance)

@for_all_robot_nuisance_pairs
def check_nuisances_obs_left_approx(id_robot, robot, id_nuisance, nuisance):  # @UnusedVariable
    check_left_inverse_approx(robot.get_spec().get_observations(), nuisance)



@for_all_robot_nuisance_pairs
def check_nuisances_obs2(id_robot, robot, id_nuisance, nuisance):  # @UnusedVariable
    check_conversions_upper_lower(robot.get_spec().get_observations(), nuisance)


def check_conversions_upper_lower(stream_spec1, nuisance):
    try:
        stream_spec2 = nuisance.transform_spec(stream_spec1)
    except UnsupportedSpec as e:
        logger.info('Skipping %s/%s because incompatible: %s' % 
                    (stream_spec1, nuisance, e))
        return Skipped('UnsupportedSpec')
    
    streamels = stream_spec1.get_streamels() 
    upper = streamels['upper']
    lower = streamels['lower']
    stream_spec1.check_valid_value(lower)
    stream_spec1.check_valid_value(upper)

    upper2 = nuisance.transform_value(upper)
    lower2 = nuisance.transform_value(lower)
    stream_spec2.check_valid_value(upper2)
    stream_spec2.check_valid_value(lower2)
    
def check_inverse(stream_spec1, nuisance):
    # print('Checking %s / %s ' % (stream_spec1, nuisance))
    nuisance_inv = None
    try:
        try:
            stream_spec2 = nuisance.transform_spec(stream_spec1)
        except UnsupportedSpec as e:
            logger.info('Skipping %s/%s because incompatible: %s' % 
                        (stream_spec1, nuisance, e))
            return Skipped('UnsupportedSpec')

        value1 = stream_spec1.get_random_value()
        stream_spec1.check_valid_value(value1)

        value2 = nuisance.transform_value(value1)
        stream_spec2.check_valid_value(value2)

        try:
            nuisance_inv = nuisance.inverse()
        except NuisanceNotInvertible as e:
            logger.info('Skipping some tests %s/%s because not invertible:'
                        ' %s' % (stream_spec1, nuisance, e))
            return Skipped('NuisanceNotInvertible')

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

def check_left_inverse(stream_spec1, nuisance):
    nuisance_left_inv = None
    try:
        try:
            stream_spec2 = nuisance.transform_spec(stream_spec1)
        except UnsupportedSpec as e:
            logger.info('Skipping %s/%s because incompatible: %s' % 
                        (stream_spec1, nuisance, e))
            return Skipped('UnsupportedSpec')
    
        # print('Checking %s / %s ' % (stream_spec1, nuisance))
    
        value1 = stream_spec1.get_random_value()
        stream_spec1.check_valid_value(value1)
    
        value2 = nuisance.transform_value(value1)
        stream_spec2.check_valid_value(value2)
    
        try:
            nuisance_left_inv = nuisance.left_inverse()
        except NuisanceNotLeftInvertible:
            return Skipped('NotLeftInvertible')
        
        assert isinstance(nuisance_left_inv, RepresentationNuisance)
        
        try:
            stream_spec1b = nuisance_left_inv.transform_spec(stream_spec2)
        except UnsupportedSpec as e:
            msg = ('The inverse of the nuisance does not seem to be able '
                   'to handle the result:\n%s\n\n'
                   '  stream_spec1: %s\n'
                   '      nuisance: %s\n'
                   '  stream_spec2: %s\n'
                   '  nuisance_left_inv: %s\n' % 
                   (indent(str(e), '>'),
                    stream_spec1.to_yaml(), nuisance,
                    stream_spec2.to_yaml(), nuisance_left_inv))
            raise ValueError(msg)
    
        try:
            StreamSpec.check_same_spec(stream_spec1, stream_spec1b)
        except Exception as e:
            msg = ('The inverse of the nuisance does not recreate the '
                   'initial spec:\n%s\n\n'
                   '  stream_spec1: %s\n'
                   '      nuisance: %s\n'
                   '  stream_spec2: %s\n'
                   '  nuisance_left_inv: %s\n'
                   ' stream_spec1b: %s\n' % 
                   (indent(str(e), '>'),
                    stream_spec1.to_yaml(), nuisance,
                    stream_spec2.to_yaml(), nuisance_left_inv,
                    stream_spec1b.to_yaml()))
            raise ValueError(msg)
    
        value1b = nuisance_left_inv.transform_value(value2)
        stream_spec1.check_valid_value(value1b)

    
    # consider this:
    #   value1 -> |N| -> value2 -> |N_left_inv| -> value1b -> |N| -> value2b
    # Now, value1b == value only if exact. 
    # However value2b == value2
    
        value2b = nuisance.transform_value(value1b)
        try:
            rtol = 1e-5
            atol = 1e-5
            assert_allclose(value2, value2b, rtol=rtol, atol=atol)
        except AssertionError:
            logger.error('value2 = %s' % describe_value(value2))
            logger.error('value2b = %s' % describe_value(value2b))
            logger.error('difference: %s' % (value2-value2b))
            raise
        
    except:
        logger.error('Error while testing:')
        logger.error(' stream_spec:  %s ' % stream_spec1.to_yaml())
        logger.error(' nuisance:     %s' % nuisance)
        logger.error(' nuisance_inv: %s' % nuisance_left_inv)
        raise


def check_left_inverse_approx(stream_spec1, nuisance):
    nuisance_left_inv_approx = None
    try:
        try:
            stream_spec2 = nuisance.transform_spec(stream_spec1)
        except UnsupportedSpec as e:
            logger.info('Skipping %s/%s because incompatible: %s' % 
                        (stream_spec1, nuisance, e))
            return Skipped('UnsupportedSpec')
    
        # print('Checking %s / %s ' % (stream_spec1, nuisance))
    
        value1 = stream_spec1.get_random_value()
        stream_spec1.check_valid_value(value1)
    
        value2 = nuisance.transform_value(value1)
        stream_spec2.check_valid_value(value2)
    
        try:
            nuisance_left_inv_approx = nuisance.left_inverse_approx()
        except (NuisanceNotInvertible, NuisanceNotLeftInvertible):
            raise Exception('left_inv_approx should not raise NuisanceNotLeftInvertible')
        
        assert isinstance(nuisance_left_inv_approx, RepresentationNuisance)
        
        try:
            stream_spec1b = nuisance_left_inv_approx.transform_spec(stream_spec2)
        except UnsupportedSpec as e:
            msg = ('The inverse of the nuisance does not seem to be able '
                   'to handle the result:\n%s\n\n'
                   '  stream_spec1: %s\n'
                   '      nuisance: %s\n'
                   '  stream_spec2: %s\n'
                   '  nuisance_left_inv: %s\n' % 
                   (indent(str(e), '>'),
                    stream_spec1.to_yaml(), nuisance,
                    stream_spec2.to_yaml(), nuisance_left_inv_approx))
            raise ValueError(msg)
    
        try:
            StreamSpec.check_same_spec(stream_spec1, stream_spec1b)
        except Exception as e:
            msg = ('The inverse of the nuisance does not recreate the '
                   'initial spec:\n%s\n\n'
                   '  stream_spec1: %s\n'
                   '      nuisance: %s\n'
                   '  stream_spec2: %s\n'
                   '  nuisance_left_inv: %s\n'
                   ' stream_spec1b: %s\n' % 
                   (indent(str(e), '>'),
                    stream_spec1.to_yaml(), nuisance,
                    stream_spec2.to_yaml(), nuisance_left_inv_approx,
                    stream_spec1b.to_yaml()))
            raise ValueError(msg)
    
        value1b = nuisance_left_inv_approx.transform_value(value2)
        stream_spec1.check_valid_value(value1b)

    
    # consider this:
    #   value1 -> |N| -> value2 -> |N_left_inv| -> value1b -> |N| -> value2b
    # Now, value1b == value only if exact. 
    # However value2b == value2
    
        value2b = nuisance.transform_value(value1b)
        
        # ok, the difference is here we don't check this
        # while we do for left_invertible
#         try:
#             rtol = 1e-5
#             atol = 1e-5
#             assert_allclose(value2, value2b, rtol=rtol, atol=atol)
#         except AssertionError:
#             logger.error('value2 = %s' % describe_value(value2))
#             logger.error('value2b = %s' % describe_value(value2b))
#             logger.error('difference: %s' % (value2-value2b))
#             raise
#         
    except:
        logger.error('Error while testing:')
        logger.error(' stream_spec:  %s ' % stream_spec1.to_yaml())
        logger.error(' nuisance:     %s' % nuisance)
        logger.error(' nuisance_inv: %s' % nuisance_left_inv_approx)
        raise


