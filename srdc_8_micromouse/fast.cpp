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

// 距離len[mm]を初速v0から最高速vmaxまで加速し、終速v1まで減速して走る時間[us]。a[mm/s^2]。
// run.cppのstraight()/accelerate()/decelerate()と同じ台形の速度変化とみなす(設計メモ §11)
static float trapTime(float len, float v0, float v1, float vmax, float a)
{
  if (len <= 0.0f) return 0.0f;
  v0 = max(v0, (float)MIN_SPEED);
  v1 = max(v1, (float)MIN_SPEED);
  vmax = max(vmax, max(v0, v1));
  const float d_acc = (vmax * vmax - v0 * v0) / (2.0f * a);
  const float d_dec = (vmax * vmax - v1 * v1) / (2.0f * a);
  if (d_acc + d_dec <= len) {
    return ((vmax - v0) / a + (vmax - v1) / a + (len - d_acc - d_dec) / vmax) * 1.0e6f;
  }
  const float vp = sqrtf((2.0f * a * len + v0 * v0 + v1 * v1) / 2.0f);
  if (vp < max(v0, v1)) {
    return len * 2.0f / (v0 + v1) * 1.0e6f;  // v0からv1まで加減速し切れない短い距離。平均速度で近似する
  }
  return ((vp - v0) / a + (vp - v1) / a) * 1.0e6f;
}

// 走行パラメータから経路探索の重み(各動作の時間)を作る。slalom=trueでrunSlalom()用(設計メモ §11)
static void costMake(bool slalom, t_path_cost * c)
{
  const float a = g_run.search_accel * 1000.0f;
  const float v_stop = (float)g_run.stoppableSpeed(HALF_SECTION);  // 半区画で止まり切れる速度
  const float v_c = slalom ? (float)g_run.slalom_speed : v_stop;   // 直進区間の始めと終わりの速度
  const float v_max = (float)g_run.max_speed;
  const float wait = STOP_WAIT_MS * 1000.0f;  // stop()の待ち時間

  // 超信地旋回(rotate())。片輪がtread_width×π/4×times走る間をturn_accelで加減速する
  auto rotTime = [&](int times) {
    return trapTime(
             g_run.tread_width * PI / 4.0f * times, MIN_SPEED, MIN_SPEED, g_run.search_speed,
             g_run.turn_accel * 1000.0f) +
           wait;
  };

  c->straight[0] = 0;
  c->goal[0] = (uint32_t)(trapTime(HALF_SECTION, v_c, MIN_SPEED, v_c, a) + wait);
  for (int k = 1; k < MAZESIZE_MAX; k++) {
    c->straight[k] = (uint32_t)trapTime(k * SECTION, v_c, v_c, v_max, a);
    c->goal[k] = (uint32_t)(trapTime(k * SECTION, v_c, v_stop, v_max, a) +
                            trapTime(HALF_SECTION, v_stop, MIN_SPEED, v_stop, a) + wait);
  }
  if (slalom) {
    // slalomCell()は区画境界から次の区画境界までをほぼslalom_speedで走る
    c->turn = (uint32_t)(SECTION / v_c * 1.0e6f);
  } else {
    // decelerate(HALF) → rotate(90度) → accelerate(HALF)
    c->turn = (uint32_t)(trapTime(HALF_SECTION, v_c, MIN_SPEED, v_c, a) + wait + rotTime(1) +
                         trapTime(HALF_SECTION, MIN_SPEED, v_c, v_c, a));
  }
  c->rotate[0] = 0;
  c->rotate[1] = (uint32_t)rotTime(1);
  c->rotate[2] = (uint32_t)rotTime(2);
}

// 経路を作ってSerialへ出す。ゴールへ行けなければ点滅で知らせてfalse(設計メモ §8・§11)
static bool fastPathPrepare(bool slalom, short gx, short gy, short gx2, short gy2)
{
  t_path_cost cost;
  uint32_t total_us;
  costMake(slalom, &cost);
  if (!g_map.fastPathMake(gx, gy, gx2, gy2, cost, &total_us)) {
    Serial.printf("fast path: goal unreachable\r\n");
    g_map.unreachableWait();
    return false;
  }
  Serial.printf(
    "fast path (%s): turn %lu ms, est. %lu ms\r\n", slalom ? "slalom" : "pivot",
    (unsigned long)(cost.turn / 1000), (unsigned long)(total_us / 1000));
  g_map.fastPathPrint();
  return true;
}

// 既知の迷路情報から走行時間が最小の経路(ダイクストラ法)を作り、直進区間をまとめて走る
void FAST::run(short gx, short gy, short gx2, short gy2)
{
  t_global_direction glob_nextdir;
  int straight_count = 0;
  int turn_speed = g_run.stoppableSpeed(HALF_SECTION);  // 旋回・ゴール前後の速度

  if (!fastPathPrepare(false, gx, gy, gx2, gy2)) return;

  // 次の方向へ向きを合わせる
  switch (g_map.fastPathNext(&glob_nextdir)) {
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
      // 経路の列が尽きた(通常は起きない)。motorDisable()はfastPathNext()の中で実行済み(設計メモ §8)
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
    switch (g_map.fastPathNext(&glob_nextdir)) {
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
        // motorDisable()はfastPathNext()の中で実行済み(設計メモ §8)
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
  g_map.wallSet(g_sensor.sen_f.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
  g_run.decelerate(HALF_SECTION, turn_speed);
}

// 既知の迷路情報から直進区間をまとめて最短走行(スラローム版)。90度旋回はslalomCell()、
// 行き止まりはuTurn()、開始直後の方向合わせのみrotate()。詳細は設計メモ §3参照
void FAST::runSlalom(short gx, short gy, short gx2, short gy2)
{
  t_global_direction glob_nextdir;
  int straight_count = 0;
  const int stop_speed = g_run.stoppableSpeed(HALF_SECTION);  // ゴール・Uターン前の速度

  if (!fastPathPrepare(true, gx, gy, gx2, gy2)) return;

  // 次の方向へ向きを合わせる(停止状態のためrotate()を使う)
  switch (g_map.fastPathNext(&glob_nextdir)) {
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
      // 経路の列が尽きた(通常は起きない)。motorDisable()はfastPathNext()の中で実行済み(設計メモ §8)
      return;
    default:
      break;
  }

  g_run.startAccelerate(HALF_SECTION, g_run.slalom_speed);
  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  // 曲がり角までの区間をまとめて走行する
  while (!g_map.posIn(gx, gy, gx2, gy2)) {
    const t_local_direction dir = g_map.fastPathNext(&glob_nextdir);
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
        // 既知の壁だけの経路では通常は起きない(設計メモ §11)
        if (straight_count > 0) {
          g_run.straight(straight_count * SECTION, g_run.speed, g_run.max_speed, stop_speed);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, g_run.speed);
        g_search.turnBack(
          g_sensor.sen_f.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
        g_run.accelerate(HALF_SECTION, g_run.slalom_speed);
        break;
      case loca_dir_error:
        // motorDisable()はfastPathNext()の中で実行済み(設計メモ §8)
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
  // ゴール区画の壁を記録する(run()と同じ、設計メモ §11)。側壁は復路のモード3と同じth_wall_slalomで判定
  g_map.wallSet(
    g_sensor.sen_f.is_wall, g_sensor.sen_r.value > g_sensor.sen_r.th_wall_slalom,
    g_sensor.sen_l.value > g_sensor.sen_l.th_wall_slalom);
  g_run.decelerate(HALF_SECTION, g_run.speed);
}
