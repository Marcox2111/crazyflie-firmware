/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mm_pose.h"
#include "math3d.h"
#include "debug.h"

int functionCallCounter = 0;

void kalmanCoreUpdateWithPose(kalmanCoreData_t* this, poseMeasurement_t *pose)
{
  functionCallCounter++;
  // a direct measurement of states x, y, and z, and orientation
  // do a scalar update for each state, since this should be faster than updating all together
  for (int i=0; i<3; i++) {
    float h[KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_X+i] = 1;
    kalmanCoreScalarUpdate(this, &H, pose->pos[i] - this->S[KC_STATE_X+i], pose->stdDevPos);
  }

  // compute orientation error
  struct quat const q_ekf = mkquat(this->q[1], this->q[2], this->q[3], this->q[0]);
  struct quat const q_measured = mkquat(pose->quat.x, pose->quat.y, pose->quat.z, pose->quat.w);
  struct quat const q_residual = qqmul(qinv(q_ekf), q_measured);
  // small angle approximation, see eq. 141 in http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
  // struct vec const err_quat = vscl(2.0f / q_residual.w, quatimagpart(q_residual));
  struct vec const err_quat=quat2rpy(q_residual);
  // struct vec const att_ekf = quat2rpy(q_ekf);
  // struct vec const att_meas = quat2rpy(q_measured);
  // struct vec const err_quat2 = vsub(att_meas, att_ekf);

  // if (functionCallCounter % 5 == 0) {
  //   // DEBUG_PRINT("mYaw %f, eYaw %f, errQYaw %f, errYaw %f \n", (double)att_meas.z, (double)att_ekf.z, (double)err_quat.z, (double)err_quat2.z);
  //   DEBUG_PRINT("eq0 %f, eq1 %f, eq2 %f, eq3 %f \n", (double)q_ekf.w, (double)q_ekf.x, (double)q_ekf.y,(double)q_ekf.z);
  //   DEBUG_PRINT("mq0 %f, mq1 %f, mq2 %f, mq3 %f \n", (double)q_measured.w, (double)q_measured.x, (double)q_measured.y, (double)q_measured.z);
  // }
  // do a scalar update for each state
  {
    float h[KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_D0] = 1;
    kalmanCoreScalarUpdate(this, &H, err_quat.x, pose->stdDevQuat);
    h[KC_STATE_D0] = 0;

    h[KC_STATE_D1] = 1;
    kalmanCoreScalarUpdate(this, &H, err_quat.y, pose->stdDevQuat);
    h[KC_STATE_D1] = 0;

    h[KC_STATE_D2] = 1;
    kalmanCoreScalarUpdate(this, &H, err_quat.z, pose->stdDevQuat);
    h[KC_STATE_D2] = 0;
  }
}
