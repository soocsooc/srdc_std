// Copyright 2025 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SRC_RUN_H_
#define SRC_RUN_H_

#include "map_manager.h"  // t_local_direction

typedef struct
{
  double control;
  double error;
  double kp;
  bool enable;
} t_control;

typedef enum { MOT_FORWARD = 1, MOT_BACK = 2 } t_CW_CCW;

class RUN
{
private:
public:
  volatile double accel;
  volatile double speed;
  volatile double speed_target_r;
  volatile double speed_target_l;
  volatile double upper_speed_limit;
  volatile double lower_speed_limit;
  float search_accel;
  short search_speed;
  short max_speed;
  short slalom_speed;  // slalom()の旋回中の速度[mm/s](外輪の基準)
  float turn_accel;
  // slalom()の旋回の形(設計メモ §3)。内輪の最低速度比、外輪の増速率、fを0→1にする外輪の走行距離[mm]
  float slalom_inner_ratio;
  float slalom_outer_gain;
  float slalom_ramp_len;
  float slalom_offset;  // 90度旋回の前後の張り出し[mm](slalomParamUpdate()で計算)
  float slalom_ramp_th;  // fを0→1へ上げる間に曲がる角度[rad](slalomParamUpdate()で計算)
  float slalom_tail;  // 直前のslalom()で外輪がs_totalを超えて走った距離[mm](確認用)
  short uturn_back_len;  // uTurn()のバック距離[mm]
  short wall_return_len;  // 壁に当てた後、区画中心へ戻る前進距離[mm](uTurn()・走行開始時)
  short uturn_speed;  // uTurn()のバック/前進の速度[mm/s]
  t_control con_wall;

  float tire_diameter;
  float tread_width;
  float pulse;

  RUN();
  void interrupt(void);
  void counterClear(void);
  void straight(int len, int init_speed, int max_sp, int finish_speed);
  void accelerate(int len, int finish_speed);
  void oneStep(int len, int init_speed);
  void decelerate(int len, float init_speed);
  int stoppableSpeed(int len);
  void rotate(t_local_direction dir, int times);
  void slalom(t_local_direction dir, float angle);
  // 区画境界から次の区画境界までを1動作で90度曲がる(区画中心の前後slalom_offsetで旋回、設計メモ §3)
  void slalomCell(t_local_direction dir);
  // スラロームの形のパラメータの範囲チェックとslalom_offsetの再計算
  void slalomParamUpdate(void);
  void uTurn(t_local_direction dir);
  // 前センサの値がthを超えたら区画境界に着いたとみなすoneStep()。max_lenは距離の上限。
  // debug_log=trueでログを出し、スイッチで中断できる(中断したらtrue、設計メモ §4)
  bool oneStepDynamic(int max_len, int init_speed, int th, bool debug_log = false);
  // 走行開始時に位置を合わせる。後ろの壁に当ててからwall_return_len前進する。
  // uTurn()と同じuturn_back_len/wall_return_len/uturn_speedを使う(設計メモ §6)
  void alignStartPosition(void);
  // 走行開始時の加速。align=trueなら先にalignStartPosition()で区画中心へ合わせる(設計メモ §6)
  void startAccelerate(int len, int finish_speed, bool align = true);

private:
  int step_lr_len, step_lr;
  int step_l, step_r;
  volatile double turn_f;  // 旋回の強さ(0〜1)。interrupt()が外輪・内輪の速度に反映する
  volatile bool turn_enable;  // trueの間、interrupt()がturn_fで左右速度に差をつける
  int turn_sign;              // +1:左旋回方向, -1:右旋回方向
  void dirSet(t_CW_CCW dir_left, t_CW_CCW dir_right);
  void speedSet(double l_speed, double r_speed);
  void stepGet(void);
  void stepGetLR(void);
  void stay(float l_speed);
  void stop(void);
  void moveFixed(t_CW_CCW dir, int len, int spd);
  double slalomRampAngle(double s_end);  // fを上げる区間で外輪が0→s_end[mm]走る間の旋回角[rad]
  double slalomLength(double obj_th);  // obj_th[rad]曲がるスラロームの外輪の走行距離[mm]
};

extern RUN g_run;

void controlInterrupt(void);

#endif /* SRC_RUN_H_ */
