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
    // 現在区画の壁情報を記録する
    g_map.wallSet(g_sensor.sen_f.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);

    // ゴール到達、またはスイッチ押下で探索終了
    if (g_map.posIn(gx, gy, gx2, gy2) || (switchGet() != 0)) {
      break;
    }

    if (g_sensor.sen_l.is_wall == false) {
      // 左が開いている → 左折して1区画進む
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(left, 1);
      g_map.nextDir(left);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
      g_map.axisUpdate();
    } else if (g_sensor.sen_f.is_wall == false) {
      // 前が開いている → 直進して1区画進む
      g_run.oneStep(SECTION, g_run.search_speed);
      g_map.axisUpdate();
    } else if (g_sensor.sen_r.is_wall == false) {
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

  // 半区画で減速停止（迷路情報の保存は呼び出し元の modeExec で行う）
  g_run.decelerate(HALF_SECTION, g_run.search_speed);
}

