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

#ifndef SRC_SENSOR_H_
#define SRC_SENSOR_H_

#include "run.h"

typedef struct
{
  short value;
  short error;
  short ref;
  short th_wall;
  short th_control;
  // モード9専用の側方壁判定しきい値(sen_r/sen_lのみ使用)。詳細は設計メモ §5参照。
  short th_wall_slalom;
  bool is_wall;
  bool is_control;
} t_sensor;

// REF・しきい値のベース値(Flash・Webで編集する値)。判定に使う値はapplyRatio()で倍率を掛けて作る(設計メモ §12)
typedef struct
{
  short ref_r, ref_l;
  short th_r, th_l, th_fr, th_fl;
  short th_r_slalom, th_l_slalom;
  short th_approach_sum, th_approach_sum_slalom;
  // 調整モード10の測定値(区画中央、1区画先の前壁あり/なし)。th_fr/flはこの平均。判定には使わない
  short fr_wall, fl_wall, fr_nowall, fl_nowall;
} t_sensor_base;

class SENSOR
{
public:
  volatile t_sensor sen_r, sen_l, sen_fr, sen_fl;
  volatile short battery_value;
  // oneStepDynamic()の前壁到達判定閾値(FR+FLの和、モード8用/モード9用)。和にするのは車体の
  // 向きの影響を打ち消すため。詳細は設計メモ §4参照。
  short th_approach_sum;
  short th_approach_sum_slalom;
  t_sensor_base base;
  short ratio_pct;  // baseに掛ける倍率[%]
  // SLED点灯窓が伸びて棄却したサンプル数 (ch: 0=FR 1=FL 2=R 3=L)。
  // SLED_WINDOW_MAX_US のチューニング用。
  volatile unsigned long sample_reject[4];
  void interrupt(void);
  void applyRatio(void);  // base×ratio_pctを判定用の値へ反映する
};

extern SENSOR g_sensor;

void sensorInterrupt(void);

#endif /* SRC_SENSOR_H_ */