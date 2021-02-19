import numpy as np
from math import sin, cos

class fowardKinematics:
    #DH matrix in format [[theetas], [alphas], [rs], [ds]]
    def __init__(self, dh_matrix):
        self.dh_matrix = dh_matrix
        self.createMatrices(self.dh_matrix)
    
    #Generates DH matrix
    def createMatrices(self, dh_matrix):
        self.joint_matrix = []

        for i in range(self.dh_matrix.shape[0]):    #Generates the matrix of DH matrices
            self.joint_matrix.append(np.zeros((4,4)))
            
            self.joint_matrix[-1][0][0] = cos(dh_matrix[i][0])
            self.joint_matrix[-1][0][1] = -sin(dh_matrix[i][0])*cos(dh_matrix[i][1])
            self.joint_matrix[-1][0][2] = sin(dh_matrix[i][0])*cos(dh_matrix[i][1])
            self.joint_matrix[-1][0][3] = dh_matrix[i][2]*cos(dh_matrix[i][0])
                             
            self.joint_matrix[-1][1][0] = sin(dh_matrix[i][0])
            self.joint_matrix[-1][1][1] = cos(dh_matrix[i][0])*cos(dh_matrix[i][1])
            self.joint_matrix[-1][1][2] = -cos(dh_matrix[i][0])*sin(dh_matrix[i][1])
            self.joint_matrix[-1][1][3] = dh_matrix[i][2]*sin(dh_matrix[i][0])
                             
            self.joint_matrix[-1][2][0] = 0
            self.joint_matrix[-1][2][1] = -sin(dh_matrix[i][1])
            self.joint_matrix[-1][2][2] = cos(dh_matrix[i][1])
            self.joint_matrix[-1][2][3] = dh_matrix[i][3]
                             
            self.joint_matrix[-1][3][0] = 0
            self.joint_matrix[-1][3][1] = 0
            self.joint_matrix[-1][3][2] = 0
            self.joint_matrix[-1][3][3] = 1
        

    def transformation(self, A, B): #Transforms from matrix A to matrix B
        return np.matmul(A,B)

    def findJointPose(self, joint): #Gives transformation from robot base to selected joint
        if joint == 1:
            return self.joint_matrix[0]
        else:
            return self.transformation(self.findJointPose(joint-1),self.joint_matrix[joint-1])
        