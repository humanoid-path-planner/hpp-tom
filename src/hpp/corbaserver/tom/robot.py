#
# Copyright (c) 2016 CNRS
# Authors: Florent Lamiraux
#
# This file is part of hpp-tom
# hpp-tom is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-tom is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-tom  If not, see
# <http://www.gnu.org/licenses/>.

from math import pi
from hpp.corbaserver.robot import Robot as Parent

##
#  Control of robot Tom in hpp
#
#  This class implements a client to the corba server implemented in
#  hpp-corbaserver. It derive from class hpp.corbaserver.robot.Robot.
#
#  This class is also used to initialize a client to gepetto-viewer in order to
#  display configurations of the Tom robot.
#
#  At creation of an instance, the urdf and srdf files are loaded using
#  idl interface hpp::corbaserver::Robot::loadRobotModel.
class Robot (Parent):
    ##
    #  Information to retrieve urdf and srdf files.
    packageName = "tom_description"
    meshPackageName = "tom_description"
    rootJointType = "planar"
    urdfName = "tom_full"
    urdfSuffix = ""
    srdfSuffix = ""
    halfSitting = [0.0, 0.0, 1.0, 0.0, -pi/2, -pi/4, 0, -pi/6, -2*pi/3, -pi/4,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.8295, 0, 0, 0, pi/2,
                   -3*pi/4, 0, -5*pi/6, 2*pi/3, pi/4, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0.8295, 0, 0, 0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
                   1.0, 0.0]
    openLeftHand = {
        'l_hand_joint_index_0' : 0,
        'l_hand_joint_index_1' : 0,
        'l_hand_joint_index_2' : 0,
        'l_hand_joint_index_3' : 0,
        'l_hand_joint_middle_0' : 0,
        'l_hand_joint_middle_1' : 0,
        'l_hand_joint_middle_2' : 0,
        'l_hand_joint_middle_3' : 0,
        'l_hand_joint_ring_0' : 0,
        'l_hand_joint_ring_1' : 0,
        'l_hand_joint_ring_2' : 0,
        'l_hand_joint_ring_3' : 0,
        'l_hand_joint_thumb_0' : 0.8295,
        'l_hand_joint_thumb_1' : 0,
        'l_hand_joint_thumb_2' : 0,
        'l_hand_joint_thumb_3' : 0,
    }
    openRightHand = {
        'r_hand_joint_index_0' : 0,
        'r_hand_joint_index_1' : 0,
        'r_hand_joint_index_2' : 0,
        'r_hand_joint_index_3' : 0,
        'r_hand_joint_middle_0' : 0,
        'r_hand_joint_middle_1' : 0,
        'r_hand_joint_middle_2' : 0,
        'r_hand_joint_middle_3' : 0,
        'r_hand_joint_ring_0' : 0,
        'r_hand_joint_ring_1' : 0,
        'r_hand_joint_ring_2' : 0,
        'r_hand_joint_ring_3' : 0,
        'r_hand_joint_thumb_0' : 0.8295,
        'r_hand_joint_thumb_1' : 0,
        'r_hand_joint_thumb_2' : 0,
        'r_hand_joint_thumb_3' : 0,
    }

    closedLeftHand = {
        'l_hand_joint_index_0' : 0.,
        'l_hand_joint_index_1' : 0.95,
        'l_hand_joint_index_2' : 0.9558,
        'l_hand_joint_index_3' : 0.88,
        'l_hand_joint_middle_0' : 0.,
        'l_hand_joint_middle_1' : 0.95,
        'l_hand_joint_middle_2' : 0.9558,
        'l_hand_joint_middle_3' : 0.88,
        'l_hand_joint_ring_0' : 0.,
        'l_hand_joint_ring_1' : 0.95,
        'l_hand_joint_ring_2' : 0.9558,
        'l_hand_joint_ring_3' : 0.88,
        'l_hand_joint_thumb_0' : 1.2,
        'l_hand_joint_thumb_1' : 0.8,
        'l_hand_joint_thumb_2' : 0.9108,
        'l_hand_joint_thumb_3' : 0.75,
    }

    closedRightHand = {
        'r_hand_joint_index_0' : 0.,
        'r_hand_joint_index_1' : 0.95,
        'r_hand_joint_index_2' : 0.9558,
        'r_hand_joint_index_3' : 0.88,
        'r_hand_joint_middle_0' : 0.,
        'r_hand_joint_middle_1' : 0.95,
        'r_hand_joint_middle_2' : 0.9558,
        'r_hand_joint_middle_3' : 0.88,
        'r_hand_joint_ring_0' : 0.,
        'r_hand_joint_ring_1' : 0.95,
        'r_hand_joint_ring_2' : 0.9558,
        'r_hand_joint_ring_3' : 0.88,
        'r_hand_joint_thumb_0' : 1.2,
        'r_hand_joint_thumb_1' : 0.8,
        'r_hand_joint_thumb_2' : 0.9108,
        'r_hand_joint_thumb_3' : 0.75,
    }

    def __init__ (self, robotName, load = True):
        Parent.__init__ (self, robotName, self.rootJointType, load)
        self.tf_root = "base_footprint"

    ## Open hand
    #
    #  \param q configuration,
    #  \param which which hand to open, should be "left" or "right".
    #  \return configuration with hand degrees of freedom modified
    def openHand (self, q, which) :
        if which == "left":
            h = self.openLeftHand
        elif which == "right":
            h = self.openRightHand
        else :
            raise RuntimeError ("which should be 'left' or 'right'")
        for j, v in h.iteritems ():
            q [self.rankInConfiguration [j]] = v
        return q

    ## Close hand
    #
    #  \param q configuration,
    #  \param which which hand to close, should be "left" or "right".
    #  \return configuration with hand degrees of freedom modified
    def closedHand (self, q, which) :
        if which == "left":
            h = self.closedLeftHand
        elif which == "right":
            h = self.closedRightHand
        else :
            raise RuntimeError ("which should be 'left' or 'right'")
        for j, v in h.iteritems ():
            q [self.rankInConfiguration [j]] = v
        return q
