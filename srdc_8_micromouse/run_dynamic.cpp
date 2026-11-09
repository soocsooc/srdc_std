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

// 前センサの値で区画境界への到達を判定する走行(設計メモ §4)。距離で判定する走行はrun.cpp

#include "pico_common.h"

// 前センサの値がthを超えるか、max_lenに達したら終えるoneStep()(設計メモ §4)。
// debug_log=trueでログを出し、スイッチですぐstop()できる。中断したらtrueを返す
bool RUN::oneStepDynamic(int max_len, int init_speed, int th, bool debug_log)
{
  int obj_step;
  unsigned long last_log_ms = 0;
  bool aborted = false;

  controlInterruptStop();
  accel = 0.0;
  speed = lower_speed_limit = upper_speed_limit = (double)init_speed;
  con_wall.enable = true;
  dirSet(MOT_FORWARD, MOT_FORWARD);
  speedSet(speed, speed);
  obj_step = (int)((float)max_len * 2.0 / pulse);
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);

    const int f = g_sensor.sen_f.value;
    if (debug_log && ((millis() - last_log_ms) >= 20)) {
      last_log_ms = millis();
      Serial.printf("len=%d f=%d\r\n", step_lr_len, f);
    }

    if (debug_log && (switchGet() != 0)) {
      aborted = true;
      break;
    }
    if (step_lr > obj_step) {
      break;  // 前壁がない区画などでは距離の上限で終える
    }
    if (f > th) {
      break;  // 前センサの値で到達を判定
    }
  }

  if (aborted) {
    stop();
  } else {
    stay(init_speed);
  }
  return aborted;
}

// 走行開始時に位置を合わせる。後ろの壁に当ててから前進する。
// uTurn()と同じuturn_back_len/wall_return_len/uturn_speedを使う(設計メモ §6)
void RUN::alignStartPosition(void)
{
  moveFixed(MOT_BACK, uturn_back_len, uturn_speed);
  moveFixed(MOT_FORWARD, wall_return_len, uturn_speed);
}

// 走行開始時の加速。alignがtrueなら後ろ壁に当てて区画中心に合わせてから、lenの間にfinish_speedまで
// 加速する。後ろに壁がない開始(ゴールからの復路など)はalign=falseで呼ぶ(設計メモ §6)
void RUN::startAccelerate(int len, int finish_speed, bool align)
{
  if (align) alignStartPosition();
  accelerate(len, finish_speed);
}
