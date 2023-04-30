#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class DynamicReconfigure(object):

    def __init__(self, *args, **kwargs):
        self.ddr = DDynamicReconfigure("cv_cfg")

        # Slider example
        self.ddr.add_variable("test_slider1", "test", 100, 0, 1000)

        # Enum example
        enum_test = self.ddr.enum([
            self.ddr.const("Small", "int", 3, "A small constant"),
            self.ddr.const("Medium", "int", 5, "A medium constant"),
            self.ddr.const("Large", "int", 7, "A large constant")
        ], "test")
        self.ddr.add_variable("test_enum",
                              "A size parameter which is edited via an enum",
                              3,
                              3,
                              7,
                              edit_method=enum_test)

        # Min Max slider examples
        self.ddr.add_variable("test_min1", "PLACEHOLDER", 179, 0, 179)
        self.ddr.add_variable("test_max1", "PLACEHOLDER", 0, 0, 179)
        self.ddr.add_variable("test_min2", "PLACEHOLDER", 255, 0, 255)
        self.ddr.add_variable("test_max2", "PLACEHOLDER", 0, 0, 255)
        self.ddr.add_variable("test_min3", "PLACEHOLDER", 255, 0, 255)
        self.ddr.add_variable("test_max3", "PLACEHOLDER", 0, 0, 255)

        self.add_variables_to_self()
        self.ddr.start(self.dyn_rec_callback)

        super(DynamicReconfigure, self).__init__(*args, **kwargs)

    def add_variables_to_self(self):
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))

        # Update all variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config


if __name__ == '__main__':
    rospy.init_node('cv_cfg')

    # Create a D(ynamic)DynamicReconfigure
    dynam = DynamicReconfigure()

    rospy.spin()
