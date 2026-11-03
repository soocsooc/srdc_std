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
#include <Arduino.h>
#include "stepper.h"
#include "run.h"
#include "pins_config.h"

// グローバルインスタンスの生成
RUN g_run;

// RUNクラスのコンストラクタおよびメンバ関数の定義
RUN::RUN()
{
  speed = 0.0;
  accel = 0.0;
}

// 1kHzのタイマ割り込み(onTimer0)から呼ばれ、RUN::interrupt()で速度を更新する
void controlInterrupt(void) { g_run.interrupt(); }

// 1msごとに速度をaccelだけ変え、min_speed〜max_speedの範囲に収める
void RUN::interrupt(void)
{

  speed += accel; // 速度を更新

  if (speed > max_speed) {
    speed = max_speed;
  }
  if (speed < min_speed) {
    speed = min_speed;
  }
}

// 左右のモータの回転方向を設定する
void RUN::dirSet(t_CW_CCW dir_left, t_CW_CCW dir_right)
{
  stepperDirSet(dir_left == MOT_FORWARD, dir_right == MOT_FORWARD);
}

// 走行パルスのカウンタを0にする
void RUN::counterClear(void) { stepperCounterClear(); }

// 左右の速度[mm/s]をパルス周波数に変換して出力する
void RUN::speedSet(double l_speed, double r_speed)
{
  stepperHzSet(l_speed / PULSE, r_speed / PULSE);  // 速度[mm/s]をパルス周波数[Hz]に変換
}

// 左右の走行パルスの合計と走行距離を取得
void RUN::stepGet(void)
{
  int l, r;
  stepperCounterGet(&l, &r);
  step_lr = abs(l) + abs(r);
  step_lr_len = (int)((float)step_lr / 2.0 * PULSE);
}

// パルスの出力を止めてモータを止める
void RUN::stop(void) { stepperStop(); }

// 最小速度から設定速度まで加速 (走行距離[mm]、設定速度[mm/s])
void RUN::accelerate(int len, int finish_speed)
{
  int obj_step;

  accel = 1.5;
  speed = min_speed = MIN_SPEED;
  max_speed = finish_speed;
  counterClear();
  speedSet(MIN_SPEED, MIN_SPEED);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / PULSE);	// 左右の走行パルスを合算して比較するため2倍

  while (1) {
    stepGet();								// 走行パルスの取得
    speedSet(speed, speed);		// 速度の設定(L, R)
    if (step_lr > obj_step) {	// 目標走行パルスに達したらループを抜ける
      break;
    }
  }
}

// 一定速度で走行 (走行距離[mm]、設定速度[mm/s])
void RUN::oneStep(int len, int init_speed)
{
  int obj_step;

  accel = 0.0;
  max_speed = init_speed;
  speed = min_speed = init_speed;
  counterClear();
  speedSet(init_speed, init_speed);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / PULSE);

  while (1) {
    stepGet();
    speedSet(speed, speed);
    if (step_lr > obj_step) {
      break;
    }
  }
}

// 設定速度から最小速度まで減速 (走行距離[mm]、初期速度[mm/s])
void RUN::decelerate(int len, int init_speed)
{
  int obj_step;

  accel = 1.5;
  max_speed = init_speed;
  speed = min_speed = init_speed;
  counterClear();
  speedSet(init_speed, init_speed);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / PULSE);

  while (1) {
    stepGet();
    speedSet(speed, speed);
    if (
      (int)(len - step_lr_len) <
      (int)(((speed * speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * accel))) {
      break;
    }
  }

  accel = -1.5;
  min_speed = MIN_SPEED;

  while (1) {
    stepGet();
    speedSet(speed, speed);
    if (step_lr > obj_step) {
      break;
    }
  }

  stop();
}
