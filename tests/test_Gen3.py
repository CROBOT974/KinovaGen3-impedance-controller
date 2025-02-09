import os
import time
import numpy as np
import mujoco.viewer
from controller.impedance import Imp_Controller


def joint_tar(q_des,v_des,q_val,v_val, M, compensate):
  """

  :param q_des: desired joint angles
  :param v_des: desired joint velocity
  :param q_val: current joint angles
  :param v_val: current joint velocity
  :param M: mass matrix of the robot arm
  :return: joint torque
  """
  B = 20.0 * np.ones(model.nv-4)
  K = 100.0 * np.ones(model.nv-4)

  acc_des = B * (v_des - v_val) + K * (q_des - q_val)
  tau = np.dot(M, acc_des.T).T + compensate
  return tau

model = mujoco.MjModel.from_xml_path("model\scene.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)# updating the condition of the model
DURATION = 1000

# collect the mass matrix
mass_matrix = np.zeros((model.nv, model.nv), dtype=np.float64)
mujoco.mj_fullM(model, mass_matrix, data.qM)
mass_matrix_arm = mass_matrix[:7,:7].copy()
mass_matrix_arm[-1, -1] *= 5.0 # the joint of end-effector has tiny mass, which leads to delay. Therefore, it was amplified

B = 20.0 * np.ones(model.nv - 4)
K = 100.0 * np.ones(model.nv - 4)

Impedence = Imp_Controller(K ,B ,mass_matrix_arm)

desire_pos = data.qpos[:7].copy()
desire_vel = np.zeros(7)
j=0

with mujoco.viewer.launch_passive(model, data) as viewer:
  start = time.time()

  while viewer.is_running() and data.time<DURATION:
    j += np.pi/1800
    step_start = time.time()
    mujoco.mj_forward(model, data)
    qfrc_bias = data.qfrc_bias[:7].copy()

    """update the desire_pos and calculate the desire_vel (sine movement)"""
    # desire_pos_pre = desire_pos
    # desire_pos[0] = np.sin(j)
    # desire_vel[0] = (desire_pos[0] - desire_pos_pre[0]) / model.opt.timestep

    des_tau = Impedence.control(desire_pos, desire_vel, data.qpos[:7].copy(),data.qvel[:7].copy(), qfrc_bias)
    des_tau = np.append(des_tau, 0)
    for i in range(8):
      data.ctrl[i] = des_tau[i]

    mujoco.mj_step(model, data)

    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)