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

axis = np.array([[0,0,1],[0,0,1]])
angles = [[0],[0]]
joints = np.array([[[0,0,0]],[[cos(angles[0][0]),sin(angles[0][0]),0]],[[cos(angles[0][0])+cos(angles[1][0]),sin(angles[0][0])+sin(angles[1][0]),0]]])
e_pos = joints[joints.shape[0]-1]

goal = np.array([[1.4142,1.4142,0]])

n = 0
#print("Loop ", n)
print ('Coordiante of end affector: ', e_pos)
print('Distance between goal and curent: ', dist(goal[0], e_pos[0]))
print('Angles are: ')
J = np.zeros((3,joints.shape[0]-1))
for i in range(joints.shape[0]-1):
    print(angles[i][0]*180/pi)
print('')

while (dist(goal[0], e_pos[0]) > 10e-5):
    n += 1
    #print("Loop ", n)
    
    for i in range(joints.shape[0]-1):
        v = np.cross(axis[i],e_pos-joints[i])
        J[0][i] = v[0][0]
        J[1][i] = v[0][1]
        J[2][i] = v[0][2]
    
    d_angle = 0.5*np.matmul(np.transpose(J),np.transpose(goal-e_pos))
    #print('d_angle',d_angle)
    angles += d_angle
    joints = np.array([[[0,0,0]],[[cos(angles[0][0]),sin(angles[0][0]),0]],[[cos(angles[0][0])+cos(angles[1][0]),sin(angles[0][0])+sin(angles[1][0]),0]]])
    e_pos = joints[joints.shape[0]-1]
    
    #print ('Coordiante of end affector: ', e_pos)
    #print('Distance between goal and curent: ',dist(goal[0], e_pos[0]))
    #print('Angles are: ')
    #for i in range(joints.shape[0]-1):
    #    print(angles[i][0]*180/pi)

    J = np.zeros((3,joints.shape[0]-1))

print("Iterations taken: ", n)
print ('Coordiante of end affector: ', e_pos)
print('Distance between goal and curent: ',dist(goal[0], e_pos[0]))
print('Angles are: ')
for i in range(joints.shape[0]-1):
    print(angles[i][0]*180/pi)
print('')
