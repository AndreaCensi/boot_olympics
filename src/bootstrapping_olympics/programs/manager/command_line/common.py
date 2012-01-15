from ....utils import UserError


def check_mandatory(options, mandatory):
    for m in mandatory:
        if options.__dict__[m] is None:
            msg = 'Mandatory option %r not passed.' % m
            raise UserError(msg)


def check_no_spurious(args):
    if args:
        raise UserError('Spurious arguments: %s' % args)

