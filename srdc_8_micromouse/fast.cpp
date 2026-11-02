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

FAST g_fast;

// 既知の迷路情報から直進区間をまとめて最短走行
void FAST::run(short gx, short gy, short gx2, short gy2)
{
  t_global_direction glob_nextdir;
  int straight_count = 0;
  int turn_speed = g_run.stoppableSpeed(HALF_SECTION);  // 旋回・ゴール前後の速度

  // 次の方向へ向きを合わせる
  t_local_direction temp_next_dir = g_map.nextDir2Get(gx, gy, gx2, gy2, &glob_nextdir);
  switch (temp_next_dir) {
    case right:
      g_run.rotate(right, 1);
      break;
    case left:
      g_run.rotate(left, 1);
      break;
    case rear:
      g_run.rotate(right, 2);
      break;
    case loca_dir_error:
      // 壁情報の破損等でゴール到達不能と判定(motorDisable()はnextDir2Get()内で実行済み)。
      // 詳細は設計メモ §8参照
      return;
    default:
      break;
  }

  g_run.startAccelerate(HALF_SECTION, turn_speed);
  straight_count = 0;
  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  // 曲がり角までの区間をまとめて走行する
  while (!g_map.posIn(gx, gy, gx2, gy2)) {
    switch (g_map.nextDir2Get(gx, gy, gx2, gy2, &glob_nextdir)) {
      case front:
        straight_count++;
        break;
      case right:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION, turn_speed, g_run.max_speed, turn_speed);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, turn_speed);
        g_run.rotate(right, 1);
        g_run.accelerate(HALF_SECTION, turn_speed);
        break;
      case left:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION, turn_speed, g_run.max_speed, turn_speed);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, turn_speed);
        g_run.rotate(left, 1);
        g_run.accelerate(HALF_SECTION, turn_speed);
        break;
      case loca_dir_error:
        // motorDisable()はnextDir2Get()内で実行済み。詳細は設計メモ §8参照
        return;
      default:
        break;
    }
    g_map.mypos.dir = glob_nextdir;
    g_map.axisUpdate();
  }
  if (straight_count > 0) {
    g_run.straight(
      straight_count * SECTION, turn_speed, g_run.max_speed, turn_speed);
  }
  // 復路の探索が最初の向きを未知の壁で決めないよう、入ったゴール区画の壁を記録する(設計メモ §11)
  g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
  g_run.decelerate(HALF_SECTION, turn_speed);
}

// 既知の迷路情報から直進区間をまとめて最短走行(スラローム版)。90度旋回はslalomCell()、
// 行き止まりはuTurn()、開始直後の方向合わせのみrotate()。詳細は設計メモ §3参照
void FAST::runSlalom(short gx, short gy, short gx2, short gy2)
{
  t_global_direction glob_nextdir;
  int straight_count = 0;
  const int stop_speed = g_run.stoppableSpeed(HALF_SECTION);  // ゴール・Uターン前の速度

  // 次の方向へ向きを合わせる(停止状態のためrotate()を使う)
  switch (g_map.nextDir2Get(gx, gy, gx2, gy2, &glob_nextdir)) {
    case right:
      g_run.rotate(right, 1);
      break;
    case left:
      g_run.rotate(left, 1);
      break;
    case rear:
      g_run.rotate(right, 2);
      break;
    case loca_dir_error:
      // 壁情報の破損等でゴール到達不能と判定(motorDisable()はnextDir2Get()内で実行済み)。
      // 詳細は設計メモ §8参照
      return;
    default:
      break;
  }

  g_run.startAccelerate(HALF_SECTION, g_run.slalom_speed);
  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  // 曲がり角までの区間をまとめて走行する
  while (!g_map.posIn(gx, gy, gx2, gy2)) {
    const t_local_direction dir = g_map.nextDir2Get(gx, gy, gx2, gy2, &glob_nextdir);
    switch (dir) {
      case front:
        straight_count++;
        break;
      case right:
      case left:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION, g_run.speed, g_run.max_speed, g_run.slalom_speed);
          straight_count = 0;
        }
        g_run.slalomCell(dir);
        break;
      case rear:
        // 既知壁のみの経路では通常発生しない想定(設計メモ §11)
        if (straight_count > 0) {
          g_run.straight(straight_count * SECTION, g_run.speed, g_run.max_speed, stop_speed);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, g_run.speed);
        g_search.turnBack(
          g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
        g_run.accelerate(HALF_SECTION, g_run.slalom_speed);
        break;
      case loca_dir_error:
        // motorDisable()はnextDir2Get()内で実行済み。詳細は設計メモ §8参照
        return;
      default:
        break;
    }
    g_map.mypos.dir = glob_nextdir;
    g_map.axisUpdate();
  }
  if (straight_count > 0) {
    g_run.straight(straight_count * SECTION, g_run.speed, g_run.max_speed, stop_speed);
  }
  // ゴール区画の壁を記録する(run()と同じ、設計メモ §11)。側壁は復路のモード9と同じth_wall_slalomで判定
  g_map.wallSet(
    g_sensor.sen_fr.is_wall, g_sensor.sen_r.value > g_sensor.sen_r.th_wall_slalom,
    g_sensor.sen_l.value > g_sensor.sen_l.th_wall_slalom);
  g_run.decelerate(HALF_SECTION, g_run.speed);
}
