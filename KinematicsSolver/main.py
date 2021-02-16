import numpy as np
from math import sin, cos, pi, sqrt
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

def dist(x,y):
    return sqrt((((x[0]*x[0])-(y[0]*y[0]))*((x[0]*x[0])-(y[0]*y[0])))+(((x[1]*x[1])-(y[1]*y[1]))*((x[1]*x[1])-(y[1]*y[1]))))

angles = [[0]]
axis = np.array([[0,0,1]])
goal = np.array([[0,1,0]])
joints = np.array([[[0,0,0]],[[cos(angles[0][0]),sin(angles[0][0]),0]]])
e_pos = joints[1]
n = 0
print("Loop ", n)
print ('Coordiante of end affector: ', e_pos)
print('Distance between goal and curent: ',dist(goal[0], e_pos[0]))
print('Angles are: ', angles[0][0]*180/pi)

while (dist(goal[0], e_pos[0]) > 10e-5):
    n += 1
    print("Loop ", n)
    
    #print(axis)
    #print(e_pos-joints[0])
    
    J= np.cross(axis,e_pos-joints[0])
    
    #print(J)
    
    d_angle = np.matmul(goal-e_pos,np.transpose(J))
    #print(d_angle)
    angles[0][0] += d_angle[0][0]
    joints = np.array([[[0,0,0]],[[cos(angles[0][0]),sin(angles[0][0]),0]]])
    e_pos = joints[1]

    print ('Coordiante of end affector: ', e_pos)
    print('Distance between goal and curent: ',dist(goal[0], e_pos[0]))
    print('Angles are: ', angles[0][0]*180/pi)
    


