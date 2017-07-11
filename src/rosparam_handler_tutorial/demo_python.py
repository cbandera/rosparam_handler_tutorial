import rospy

from dynamic_reconfigure.server import Server
from rosparam_handler_tutorial.cfg import DemoConfig
from rosparam_handler_tutorial.param.DemoParameters import DemoParameters


class Demo:
    def timer_callback(self, timer_event):
        """
        Will be called regularly to print current configuration
        """
        rospy.loginfo("Timer callback. configurable_parameter = {}. Enum: {}"
                      .format(self.params.configurable_parameter, self.params.my_enum))

    def reconfigure_callback(self, config, level):
        """
        Called when someone reconfigures this node (or at startup)
        """
        self.params.from_config(config)
        rospy.logwarn("Parameter update:\n{}".format(self.params))
        return config

    def __init__(self):
        """
        Sets up ros stuff
        """
        # Initialization
        self.params = DemoParameters()

        # Set up dynamic reconfiguration and timer
        self.timer = rospy.Timer(rospy.Duration(1./self.params.rate), callback=self.timer_callback)
        self.reconfigure = Server(DemoConfig, self.reconfigure_callback)


