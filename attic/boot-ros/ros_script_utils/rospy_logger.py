try:
    import rospy
except:  # allow to run nose even if ros is not installed
    pass


class RospyLogger:

    def __init__(self, prefix):
        self.prefix = prefix

    def info(self, s):
        rospy.loginfo("%s:%s" % (self.prefix, s))  # @UndefinedVariable

    def debug(self, s):
        rospy.logdebug("%s:%s" % (self.prefix, s))  # @UndefinedVariable

    def error(self, s):
        rospy.logerr("%s:%s" % (self.prefix, s))  # @UndefinedVariable

