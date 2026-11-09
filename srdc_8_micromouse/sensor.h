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
  // モード3の横壁の閾値(sen_r/sen_lだけで使う、設計メモ §5)
  short th_wall_slalom;
  bool is_wall;
  bool is_control;
} t_sensor;

// REF・しきい値のベース値(Flash・Webで編集する値)。判定に使う値はapplyRatio()で倍率を掛けて作る(設計メモ §12)
typedef struct
{
  short ref_r, ref_l;
  short th_r, th_l, th_f;
  short th_r_slalom, th_l_slalom;
  short th_approach, th_approach_slalom;
  // 調整モード10の測定値(区画中央、1区画先の前壁あり/なし)。th_fはこの平均。判定には使わない
  short f_wall, f_nowall;
} t_sensor_base;

class SENSOR
{
public:
  volatile t_sensor sen_r, sen_l, sen_f;
  volatile short battery_value;
  // oneStepDynamic()で前壁が近づいたと判断する前センサの閾値(モード2用/モード3用、設計メモ §4)
  short th_approach;
  short th_approach_slalom;
  t_sensor_base base;
  short ratio_pct;  // baseに掛ける倍率[%]
  // SLED_WINDOW_MAX_USを超えて捨てた読み取りの回数(ch: 0=前 1=右 2=左)。SLED_WINDOW_MAX_USの調整用
  volatile unsigned long sample_reject[3];
  // LED点灯からAD変換が終わるまでの時間[us]の最大値(ch: 0=前 1=右 2=左)。割り込みで読み取りが
  // 遅れていないかの確認用。sled_on_us_maxは調整モード6で表示するたびに0へ戻し、sled_on_us_max_allは起動からの最大
  volatile unsigned short sled_on_us_max[3];
  volatile unsigned short sled_on_us_max_all[3];
  void interrupt(void);
  void applyRatio(void);  // base×ratio_pctを判定用の値へ反映する
};

extern SENSOR g_sensor;

void sensorInterrupt(void);

#endif /* SRC_SENSOR_H_ */