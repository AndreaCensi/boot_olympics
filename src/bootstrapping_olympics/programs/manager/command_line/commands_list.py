
#
# class Storage:
#     commands = {}
#
#
# def declare_command(name, short_usage):
#     def wrap(f):
#         f.short_usage = short_usage
#         if name in Storage.commands:
#             raise Exception('Already defined command %r.' % name)
#         Storage.commands[name] = f
#         return f
#     return wrap
