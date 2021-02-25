import numpy as np
from math import sin, cos, pi, sqrt
#import InverseKinematicsSolver
import KinematicsSolver

#import FowardKinematicsSolver
#from ursina import *

#DH matrix in format [theeta alpha r d]
##a = np.array([[0,0,0,0,0],[pi/2,pi/2,pi/2,pi/2,pi/2],[0,4,1,4,1],[-2,0,0,0,0]])

#Transpose to prep for KinematicsSolver
##a = np.transpose(a)

##solver = FowardKinematicsSolver.fowardKinematics(a)
#solver.createMatrices(solver.dh_matrix)
#print(solver.transformation(solver.transformation(solver.transformation(solver.transformation(solver.joint_matrix[0],solver.joint_matrix[1]),solver.joint_matrix[2]),solver.joint_matrix[3]),solver.joint_matrix[4]))

##print(solver.findJointPose(5))
axis = [[0,0,1],[1,0,0],[0,0,1]]
angles = [pi/3,2*pi,pi/2]
m = [[0,0,0],[pi*0,pi*0,pi*0],[1,2,2],[0,0,0]]

a = KinematicsSolver.KinematicsSolver(m)
a.calculateIK(axis, [1,1,0] ,10e-10, "JT")

a.findEndEffectorPose()

b = KinematicsSolver.KinematicsSolver([[24.29518*pi/180,0,162.88556*pi/180],[0,0,0],[1,2,2],[0,0,0]])
b.findEndEffectorPose()