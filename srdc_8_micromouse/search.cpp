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

#include "pico_common.h"

SEARCH g_search;

// 左手法で走行しながら通った区画の壁情報をg_mapに記録する。ゴール区画到達かスイッチ押下で終了。
// myposは呼び出し前にg_map.positionInit()で初期化しておくこと
void SEARCH::lefthandGetWall(char gx, char gy, char gx2, char gy2)
{
  // スタート区画から北へ半区画進み、最初の区画に入る
  g_run.startAccelerate(HALF_SECTION, g_run.search_speed);
  g_map.axisUpdate();

  while (1) {
    // 現在区画の壁情報を記録する。記録と進む方向の判断に同じ値を使うよう、ここで1回だけ読む
    const bool wall_f = g_sensor.sen_f.is_wall;
    const bool wall_r = g_sensor.sen_r.is_wall;
    const bool wall_l = g_sensor.sen_l.is_wall;
    g_map.wallSet(wall_f, wall_r, wall_l);
    ledWallShow(wall_l, wall_f, wall_r);

    // ゴール到達、またはスイッチ押下で探索終了
    if (g_map.posIn(gx, gy, gx2, gy2) || (switchGet() != 0)) {
      break;
    }

    if (wall_l == false) {
      // 左が開いている → 左折して1区画進む
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(left, 1);
      g_map.nextDir(left);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
      g_map.axisUpdate();
    } else if (wall_f == false) {
      // 前が開いている → 直進して1区画進む。前壁を見落としていたら区画中心で止まり、
      // 右か後ろへ向きを変える(設計メモ §15)
      if (!straightGuarded(g_run.search_speed)) {
        g_map.axisUpdate();
        continue;
      }
      g_map.wallSet(true, wall_r, wall_l);
      ledWallShow(wall_l, true, wall_r);
      if (wall_r == false) {
        g_run.rotate(right, 1);
        g_map.nextDir(right);
      } else {
        turnBack(true, wall_r, wall_l);  // 前壁と左壁に当てて位置を補正する
        g_map.nextDir(right);
        g_map.nextDir(right);
      }
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
      g_map.axisUpdate();
    } else if (wall_r == false) {
      // 右が開いている → 右折して1区画進む
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(right, 1);
      g_map.nextDir(right);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
      g_map.axisUpdate();
    } else {
      // 行き止まり → その場で180度旋回し、来た区画へ戻る
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
      g_map.axisUpdate();
    }
  }

  // 半区画で減速停止(迷路情報の保存は呼び出し元の modeExec で行う)
  g_run.decelerate(HALF_SECTION, g_run.search_speed);
}

// 区画境界から1区画直進する。前半(区画中心まで)は前センサを見張り、th_f_guardを超えたら
// 区画境界で前壁を見落としたとみなして区画中心で止まり、trueを返す。超えなければ次の区画境界まで
// 進んでfalseを返す(設計メモ §15)
bool SEARCH::straightGuarded(int speed)
{
  g_run.oneStepDynamic(HALF_SECTION, speed, g_sensor.th_f_guard);
  if (g_sensor.sen_f.value <= g_sensor.th_f_guard) {
    g_run.oneStep(HALF_SECTION, speed);
    return false;
  }

  // 区画中心までの残りで止まる。止まり切れないときは、FRONT_GUARD_OVERRUN_MAXまで中心を越えて減速する
  const int remain = HALF_SECTION - g_run.lastLenGet();
  const int brake = (int)(((double)speed * speed - (double)MIN_SPEED * MIN_SPEED) /
                          (2.0 * 1000.0 * g_run.search_accel));
  int len = (brake > remain) ? brake : remain;
  if (len > remain + FRONT_GUARD_OVERRUN_MAX) len = remain + FRONT_GUARD_OVERRUN_MAX;
  if (len < 1) len = 1;
  g_run.decelerate(len, speed);

  // 見落としに気付いたことを音で知らせる
  buzzerEnable(DEC_FREQ);
  delay(50);
  buzzerDisable();
  return true;
}
