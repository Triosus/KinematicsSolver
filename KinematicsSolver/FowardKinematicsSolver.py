import numpy as np

class fowardKinematics:
    def __init__(self, dh_matrix):
        self.dh_matrix = dh_matrix

    def createMatrices(self, dh_matrix):
        self.joint_matrix = []


    def transformation(self, A, B):