#!/usr/bin/env python

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class DynamicReconfigure(object):
    def __init__(self, *args, **kwargs):        
        self.ddr = DDynamicReconfigure("feature_detection_cfg")

        self.ddr.add_variable("canny_threshold1", "threshold 1 for the hypertesis",   100,    0,  1000)
        self.ddr.add_variable("canny_threshold2", "threshold 2 for the hypertesis",   200,    0,  1000)
        canny_aperture_size_enum = self.ddr.enum([ self.ddr.const("Small", "int", 3, "A small constant"),
                            self.ddr.const("Medium", "int", 5, "A medium constant"),
                            self.ddr.const("Large", "int", 7, "A large constant")],
                            "An enum to set sobel procedure aperture size")
        self.ddr.add_variable("canny_aperture_size", "A size parameter which is edited via an enum", 3, 3, 7, edit_method=canny_aperture_size_enum)

        self.ddr.add_variable("hsv_hue_min", "PLACEHOLDER",   179,    0,  179)
        self.ddr.add_variable("hsv_hue_max", "PLACEHOLDER",   0,    0,  179)
        self.ddr.add_variable("hsv_sat_min", "PLACEHOLDER",   255,    0,  255)
        self.ddr.add_variable("hsv_sat_max", "PLACEHOLDER",   0,    0,  255)
        self.ddr.add_variable("hsv_val_min", "PLACEHOLDER",   255,    0,  255)
        self.ddr.add_variable("hsv_val_max", "PLACEHOLDER",   0,    0,  255)

        self.ddr.add_variable("ksize1",  "PLACEHOLDER", 7, 0, 20)
        self.ddr.add_variable("ksize2",  "PLACEHOLDER", 7, 0, 20)
        self.ddr.add_variable("sigma",  "PLACEHOLDER", .8, 0, 10)

        self.ddr.add_variable("blocksize", "PLACEHOLDER", 11, 0, 50)
        self.ddr.add_variable("C", "PLACEHOLDER", 2, -10, 30)

        self.ddr.add_variable("ed_ksize", "PLACEHOLDER", 11, 0, 50)
        self.ddr.add_variable("erosion_iterations", "PLACEHOLDER", 1, 0, 30)
        self.ddr.add_variable("dilation_iterations", "PLACEHOLDER", 1, 0, 30)

        self.add_variables_to_self()
        self.ddr.start(self.dyn_rec_callback)

        super(DynamicReconfigure, self).__init__(*args, **kwargs)

    def add_variables_to_self(self):
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        #rospy.loginfo("Received reconf call: " + str(config))
        # Update all variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config

if __name__ == '__main__':
    rospy.init_node('feature_detection_cfg')

    # Create a D(ynamic)DynamicReconfigure
    dynam = DynamicReconfigure()

    rospy.spin()
