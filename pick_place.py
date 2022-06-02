#! /usr/bin/env python
# Copyright (c) 2016-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
import intera_interface
from intera_interface import (
    Limb,
    Gripper,
    RobotParams
)

def main():

    try:
        rospy.init_node('pick_place_py')
        limb = Limb()
        rp = RobotParams()
        valid_limbs = rp.get_limb_names()
        gripper = Gripper(valid_limbs[0])
        gripper.set_dead_zone(2.5)

        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=0.6,
                                         max_linear_accel=0.6,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        endpoint_state = limb.tip_state('right_hand')

        pose = endpoint_state.pose
        for i in range (0, 3):
            if (i == 0):   
                # Greifer auf
                gripper.open()
                pose.position.x = 0.69
                pose.position.y = 0.00
                pose.position.z = -0.01

                poseStamped = PoseStamped()
                poseStamped.pose = pose

                joint_angles = limb.joint_ordered_angles()
                waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)

                traj.append_waypoint(waypoint.to_msg())
                traj.send_trajectory(timeout=None)
                gripper.close()
                # Greifer zu
            elif (i == 1):
                pose.position.x = 0.69
                pose.position.y = 0.1
                pose.position.z = 0.3

                poseStamped = PoseStamped()
                poseStamped.pose = pose

                joint_angles = limb.joint_ordered_angles()
                waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)

                traj.append_waypoint(waypoint.to_msg())
                traj.send_trajectory(timeout=None)
            else:
                pose.position.x = 0.69
                pose.position.y = 0.2
                pose.position.z = 0.02

                poseStamped = PoseStamped()
                poseStamped.pose = pose

                joint_angles = limb.joint_ordered_angles()
                waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)

                traj.append_waypoint(waypoint.to_msg())
                traj.send_trajectory(timeout=None)
                gripper.open()
                # Greifer auf
            print('move!')
            print(i)
            print('---------------------------')
   
    
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == '__main__':
    main()
