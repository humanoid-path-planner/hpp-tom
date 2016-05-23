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

from math import sqrt
from hpp.corbaserver.tom import Robot
from hpp.gepetto import PathPlayer
#Robot.urdfName = "tom_full"

robot = Robot ('tom')
robot.setJointBounds ('base_joint_xy', [-1,1,-1,1])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
from hpp.gepetto import ViewerFactory

vf = ViewerFactory (ps)

#r.loadObstacleModel ("tom_description", "table", "table")
vf.loadObstacleModel ("hpp_tutorial", "box", "box")

box_position = [.5,0,.5,1,0,0,0]
table_position = (-0.90, -0.91, -0.42, .5, .5, .5, .5)
#r.moveObstacle ("table/table_tom_link_0", table_position)
#r.moveObstacle ("tom/box_tom_link_0", table_position)
vf.moveObstacle ("box/base_link_0", box_position)

q_init = robot.halfSitting [::]

dict_goal = {
    'l_shoulder_pan_joint': -1.62236446982,
    'l_shoulder_lift_joint': -0.741943917854,
    'l_elbow_joint': 0.952500010443,
    'l_wrist_1_joint': -1.27920005559,
    'l_wrist_2_joint': -2.22347756642,
    'l_wrist_3_joint': -1.37711410223,
    'r_shoulder_pan_joint': 1.57079632679,
    'r_shoulder_lift_joint': -2.05619449019,
    'r_elbow_joint': -0.2,
    'r_wrist_1_joint': -2.61799387799,
    'r_wrist_2_joint': 2.09439510239,
    'r_wrist_3_joint': 0.785398163397,
}

q_goal = robot.halfSitting [::]
for j, v in dict_goal.iteritems ():
    i = robot.rankInConfiguration [j]
    q_goal [i] = v

for j, v in robot.openHand (None, 0.2, 'left').iteritems ():
    ps.lockJoint (j, [v,])

for j, v in robot.openHand (None, 0.2, 'right').iteritems ():
    ps.lockJoint (j, [v,])

ps.lockJoint ('base_joint_xy', [0,0])
ps.lockJoint ('base_joint_rz', [1,0])

res = ps.applyConstraints (q_init)
if res [0]:
    q_init = res [1]
else:
    raise RuntimeError ('Failed to apply constraints on init config.')

res = ps.applyConstraints (q_goal)
if res [0]:
    q_goal = res [1]
else:
    raise RuntimeError ('Failed to apply constraints on goal config.')

robot.setCurrentConfig (q_init)
r = vf.createViewer (collisionURDF = True)
r (q_init)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.selectPathPlanner ("DiffusingPlanner")
ps.selectPathValidation("Progressive", 0.01)
#ps.selectPathValidation("Progressive", 0.05)
#ps.readRoadmap ('/home/florent/devel/fiad/data/roadmap.rdm')

# ps.solve ()

pp = PathPlayer (ps.client, r)
