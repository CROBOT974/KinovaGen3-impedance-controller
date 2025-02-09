import numpy as np

class Imp_Controller():
    def __init__(self, K, B, M):
        self.damping = B
        self.stiffness = K
        self.mass_mat = M

    def control(self, q_des, v_des, q_val, v_val, compensate):
        """

          :param q_des: desired joint angles
          :param v_des: desired joint velocity
          :param q_val: current joint angles
          :param v_val: current joint velocity
          :param compensate: corliori effect
          :return: joint torque
          """
        acc_des = self.damping * (v_des - v_val) + self.stiffness * (q_des - q_val)
        tau = np.dot(self.mass_mat, acc_des.T).T + compensate
        return tau

