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

from hpp.corbaserver.manipulation.tom import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory

class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""

robot = Robot ('tom-box', 'tom')
robot.setJointBounds ('tom/base_joint_xy', [-1,1,-1,1])

ps = ProblemSolver (robot)
fk = ViewerFactory (ps)

fk.loadObjectModel (Box, 'box')
robot.setJointBounds ('box/base_joint_xyz', [-1,1,-1,1,-1,1])

openLeftHand = []
for j, v in robot.openHand (None, .75, "left").iteritems () :
    lockedJointName = "open/"+j
    openLeftHand.append (lockedJointName)
    ps.createLockedJoint (lockedJointName, j, [v,])

openRightHand = []
for j, v in robot.openHand (None, .75, "right").iteritems () :
    lockedJointName = "open/"+j
    openRightHand.append (lockedJointName)
    ps.createLockedJoint (lockedJointName, j, [v,])

closeLeftHand = []
for j, v in robot.openHand (None, .24, "left").iteritems () :
    lockedJointName = "close/"+j
    closeLeftHand.append (lockedJointName)
    ps.createLockedJoint (lockedJointName, j, [v,])

closeRightHand = []
for j, v in robot.openHand (None, .24, "right").iteritems () :
    lockedJointName = "close/"+j
    closeRightHand.append (lockedJointName)
    ps.createLockedJoint (lockedJointName, j, [v,])

lockbox = ps.lockFreeFlyerJoint ('box/base_joint', 'box_lock')
lockBase = ps.lockPlanarJoint ('tom/base_joint', 'base_lock', [0,0,1,0])

# Build constraint graph
graph = ConstraintGraph (robot, 'graph')
graph.createNode (['hold_box_l1', 'hold_box_l2', 'hold_box_r1', 'hold_box_r2',
                   'free'])

jointNames = dict ()
jointNames ['tom'] = list ()
for j in robot.jointNames:
    if j.startswith ('tom'):
        jointNames ['tom'].append (j)

ps.addPassiveDofs ('tom', jointNames ['tom'])

graph.createGrasp ('l2_grasp', 'tom/l_finger_tip', 'box/handle2',
                   passiveJoints = 'tom')
graph.createGrasp ('l1_grasp', 'tom/l_finger_tip', 'box/handle',
                   passiveJoints = 'tom')
graph.createGrasp ('r2_grasp', 'tom/r_finger_tip', 'box/handle2',
                   passiveJoints = 'tom')
graph.createGrasp ('r1_grasp', 'tom/r_finger_tip', 'box/handle',
                   passiveJoints = 'tom')
graph.createPreGrasp ('l2_pregrasp', 'tom/l_finger_tip', 'box/handle2')
graph.createPreGrasp ('l1_pregrasp', 'tom/l_finger_tip', 'box/handle')
graph.createPreGrasp ('r2_pregrasp', 'tom/r_finger_tip', 'box/handle2')
graph.createPreGrasp ('r1_pregrasp', 'tom/r_finger_tip', 'box/handle')

graph.createWaypointEdge ('free', 'hold_box_l1', 'grasp_box_l1',
                          nb=1, weight=10)
graph.createWaypointEdge ('free', 'hold_box_l2', 'grasp_box_l2',
                          nb=1, weight=10)
graph.createWaypointEdge ('free', 'hold_box_r1', 'grasp_box_r1',
                          nb=1, weight=10)
graph.createWaypointEdge ('free', 'hold_box_r2', 'grasp_box_r2',
                          nb=1, weight=10)
graph.createEdge ('free', 'free', 'move_free', 1)
graph.createEdge ('hold_box_l1', 'hold_box_l1', 'keep_grasp_l1', 5)
graph.createEdge ('hold_box_l2', 'hold_box_l2', 'keep_grasp_l2', 5)
graph.createEdge ('hold_box_r1', 'hold_box_r1', 'keep_grasp_r1', 5)
graph.createEdge ('hold_box_r2', 'hold_box_r2', 'keep_grasp_r2', 5)

graph.setConstraints (graph=True, lockDof = openLeftHand + openRightHand +
                      lockBase)

graph.setConstraints (edge='move_free', lockDof = lockbox)

graph.setConstraints (edge='grasp_box_l1_e1', lockDof = lockbox)
graph.setConstraints (edge='grasp_box_l2_e1', lockDof = lockbox)
graph.setConstraints (edge='grasp_box_r1_e1', lockDof = lockbox)
graph.setConstraints (edge='grasp_box_r2_e1', lockDof = lockbox)

graph.setConstraints (node='grasp_box_l1_n0', pregrasps = ['l1_pregrasp',])
graph.setConstraints (node='grasp_box_l2_n0', pregrasps = ['l2_pregrasp',])
graph.setConstraints (node='grasp_box_r1_n0', pregrasps = ['r1_pregrasp',])
graph.setConstraints (node='grasp_box_r2_n0', pregrasps = ['r2_pregrasp',])

graph.setConstraints (edge='grasp_box_l1_e0', lockDof = lockbox)
graph.setConstraints (edge='grasp_box_l2_e0', lockDof = lockbox)
graph.setConstraints (edge='grasp_box_r1_e0', lockDof = lockbox)
graph.setConstraints (edge='grasp_box_r2_e0', lockDof = lockbox)

graph.setConstraints (node='hold_box_l1', grasps = ['l1_grasp',])
graph.setConstraints (node='hold_box_l2', grasps = ['l2_grasp',])
graph.setConstraints (node='hold_box_r1', grasps = ['r1_grasp',])
graph.setConstraints (node='hold_box_r2', grasps = ['r2_grasp',])

# graph.setConstraints (edge='keep_grasp_l1', lockDof = lockbox)
# graph.setConstraints (edge='keep_grasp_l2', lockDof = lockbox)
# graph.setConstraints (edge='keep_grasp_r1', lockDof = lockbox)
# graph.setConstraints (edge='keep_grasp_r2', lockDof = lockbox)

r = fk.createViewer ()

q_init = robot.halfSitting + [.75,0,.5,1,0,0,0]
res = ps.client.manipulation.problem.applyConstraints (graph.nodes ['free'], q_init)
q_init = res [1]
q_goal = robot.halfSitting + [.75,0,.3,1,0,0,0]
res = ps.client.manipulation.problem.applyConstraints (graph.nodes ['free'], q_goal)
q_goal = res [1]
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
#ps.solve ()

#Create configurations and paths by hand.
res, q1, err = ps.client.manipulation.problem.applyConstraintsWithOffset \
    (graph.edges ['grasp_box_r1_e0'], q_init, q_init)
res, i1, i2 = ps.client.manipulation.problem.buildAndProjectPath \
    (graph.edges ['grasp_box_r1_e0'], q_init, q1)
if res:
  ps.client.basic.problem.addConfigToRoadmap (q_init)
  ps.client.basic.problem.addConfigToRoadmap (q1)
  ps.client.basic.problem.addEdgeToRoadmap (q_init, q1, i2, True)
else:
  raise RuntimeError ("Failed to insert edge")

res, q2, err = ps.client.manipulation.problem.applyConstraintsWithOffset \
    (graph.edges ['grasp_box_r1_e1'], q1, q1)
res, i1, i2 = ps.client.manipulation.problem.buildAndProjectPath \
    (graph.edges ['grasp_box_r1_e1'], q1, q2)
if res:
  ps.client.basic.problem.addConfigToRoadmap (q2)
  ps.client.basic.problem.addEdgeToRoadmap (q1, q2, i2, True)
else:
  raise RuntimeError ("Failed to insert edge")

res, q3, err = ps.client.manipulation.problem.applyConstraintsWithOffset \
    (graph.edges ['grasp_box_r1_e1'], q_goal, q2)
res, i1, i2 = ps.client.manipulation.problem.buildAndProjectPath \
    (graph.edges ['keep_grasp_r1'], q2, q3)
if res:
  ps.client.basic.problem.addConfigToRoadmap (q3)
  ps.client.basic.problem.addEdgeToRoadmap (q2, q3, i2, True)
else:
  raise RuntimeError ("Failed to insert edge")

res, q4, err = ps.client.manipulation.problem.applyConstraintsWithOffset \
    (graph.edges ['grasp_box_r1_e0'], q_goal, q_goal)
res, i1, i2 = ps.client.manipulation.problem.buildAndProjectPath \
    (graph.edges ['grasp_box_r1_e0'], q3, q4)
if res:
  ps.client.basic.problem.addConfigToRoadmap (q4)
  ps.client.basic.problem.addEdgeToRoadmap (q3, q4, i2, True)
else:
  raise RuntimeError ("Failed to insert edge")

res, i1, i2 = ps.client.manipulation.problem.buildAndProjectPath \
    (graph.edges ['move_free'], q4, q_goal)
if res:
  ps.client.basic.problem.addConfigToRoadmap (q_goal)
  ps.client.basic.problem.addEdgeToRoadmap (q4, q_goal, i2, True)
else:
  raise RuntimeError ("Failed to insert edge")

pp = PathPlayer (ps.client.basic, r)
