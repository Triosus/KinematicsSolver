import numpy as np
from math import sin, cos, pi
import FowardKinematicsSolver

#a = np.array([[1,2,3],[4,5,6]])
#
#print(a)
#print(a.shape[1])
#print(a)
#[theeta alpha r d]

a = np.array([[0,0,0,0,0],[pi/2,pi/2,pi/2,pi/2,pi/2],[0,4,1,4,1],[-2,0,0,0,0]])
a = np.transpose(a)

solver = FowardKinematicsSolver.fowardKinematics(a)
#solver.createMatrices(solver.dh_matrix)
#print(solver.transformation(solver.transformation(solver.transformation(solver.transformation(solver.joint_matrix[0],solver.joint_matrix[1]),solver.joint_matrix[2]),solver.joint_matrix[3]),solver.joint_matrix[4]))

print(solver.findJointPose(5))