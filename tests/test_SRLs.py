import os
import time
import numpy as np
import mujoco.viewer
from controller.impedance import Imp_Controller

model = mujoco.MjModel.from_xml_path("model\scene4.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)# updating the condition of the model
DURATION = 1000

# collect the mass matrix
mass_matrix = np.zeros((model.nv, model.nv), dtype=np.float64)
mujoco.mj_fullM(model, mass_matrix, data.qM)
mass_matrix_arm_1 = mass_matrix[11:18,11:18].copy()
mass_matrix_arm_2 = mass_matrix[22:29,22:29].copy()
mass_matrix_arm_1[-1, -1] *= 5.0 # the joint of end-effector has tiny mass, which leads to delay. Therefore, it was amplified
mass_matrix_arm_2[-1, -1] *= 5.0

B = 20.0 * np.ones(7)
K = 100.0 * np.ones(7)

Impedence1 = Imp_Controller(K ,B ,mass_matrix_arm_1)
Impedence2 = Imp_Controller(K ,B ,mass_matrix_arm_2)

desire_pos = data.qpos.copy()
desire_vel = np.zeros(7)
j=0

with mujoco.viewer.launch_passive(model, data) as viewer:
  start = time.time()

  while viewer.is_running() and data.time<DURATION:
    step_start = time.time()
    mujoco.mj_forward(model, data)
    qfrc_bias = data.qfrc_bias.copy()  # coriolis effect

    des_tau1 = Impedence1.control(desire_pos[11:18], desire_vel, data.qpos[11:18].copy(),data.qvel[11:18].copy(), qfrc_bias[11:18])
    des_tau1 = np.append(des_tau1, 0)
    des_tau2 = Impedence2.control(desire_pos[22:29], desire_vel, data.qpos[22:29].copy(), data.qvel[22:29].copy(), qfrc_bias[22:29])
    des_tau2 = np.append(des_tau2, 0)
    for i in range(23):
      if 15> i > 6:
        data.ctrl[i] = des_tau1[i - 7]
      elif i> 14:
        data.ctrl[i] = des_tau2[i - 15]

    mujoco.mj_step(model, data)

    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)