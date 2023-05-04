#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from franka_bimanual_teaching.cfg import dual_arm_dymamic_teachingConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {Name_task}, {load_model_left}, {load_model_right}, {bool_model_bimanual}, {Active}, {Demonstration}, {Execution}, {progress_trjectory}, {progress_left}, {progress_right}, {save_model_left}, {save_model_right}, {save_model_bimanual} """.format(**config))
    return config
 
if __name__ == "__main__":
    rospy.init_node("dynamic_teaching", anonymous = False)
 
    srv = Server(dual_arm_dymamic_teachingConfig, callback)
    rospy.spin()
