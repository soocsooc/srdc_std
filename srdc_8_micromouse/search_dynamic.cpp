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

// 既存のsearch.cpp(距離カウント方式)とは別に、センサ閾値主導の到達判定(run_dynamic.cpp)を
// 使う探索走行を試すための新規ファイル。search.cpp自体は無変更。詳細はスラローム機能_実装まとめ.md §2.18参照。

#include "pico_common.h"

// デバッグ用: 認識した壁情報をLEDへ表示する(左:0b1000, 前:0b0110, 右:0b0001の論理和)。
static void ledWallShow(bool wall_l, bool wall_f, bool wall_r)
{
  ledSet((wall_l ? 0b1000 : 0) | (wall_f ? 0b0110 : 0) | (wall_r ? 0b0001 : 0));
}

// uTurn(right)は左壁→前壁、uTurn(left)は右壁→前壁へ後ろから当てるので、当てる壁が無ければ
// その場で回るだけにする(行き止まりでない区画で後ろを選んだとき、§2.70)
void SEARCH::turnBack(bool wall_f, bool wall_r, bool wall_l)
{
  if (wall_f && wall_l) {
    g_run.uTurn(right);
  } else if (wall_f && wall_r) {
    g_run.uTurn(left);
  } else {
    g_run.rotate(right, 2);
  }
}

// adachi()(search.cpp)と同一の骨格。case frontのみg_run.oneStep()をg_run.oneStepDynamic()に
// 置き換え、前方センサ閾値主導で1区画分の到達を判定する。right/left/rearの旋回は無変更。
void SEARCH::adachiDynamic(char gx, char gy, char gx2, char gy2)
{
  t_global_direction glob_nextdir;
  int cruise_speed = g_run.stoppableSpeed(HALF_SECTION);  // 半区画で止まり切れる巡航速度

  // 現在位置から最初の進行方向へ向きを合わせる
  switch (g_map.nextDirGet(gx, gy, gx2, gy2, &glob_nextdir)) {
    case front:
      break;
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
      // 壁情報の破損等でゴール到達不能と判定(motorDisable()はnextDirGet()内で実行済み)。
      // 詳細はスラローム機能_実装まとめ.md §2.17参照
      return;
  }

  // 後ろ壁に合わせるのはスタート区画から出るときだけ(復路の開始は後ろに壁がない)
  g_run.startAccelerate(SECTION / 3, cruise_speed, (g_map.mypos.x == 0) && (g_map.mypos.y == 0));
  // 助走をHALF_SECTIONからSECTION/3に短縮した分だけ区画境界の手前にいるので、その分を最初の
  // 距離基準の動作に足して境界へ合わせる(スラローム機能_実装まとめ.md §2.47参照)
  int start_short = HALF_SECTION - SECTION / 3;

  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  // センサで壁情報を更新し、次の方向へ進む
  while (!g_map.posIn(gx, gy, gx2, gy2)) {
    g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
    ledWallShow(g_sensor.sen_l.is_wall, g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall);

    switch (g_map.nextDirGet(gx, gy, gx2, gy2, &glob_nextdir)) {
      case front:
        g_run.oneStepDynamic(SECTION + start_short, cruise_speed, g_sensor.th_approach_sum);
        break;
      case right:
        g_run.decelerate(HALF_SECTION + start_short, cruise_speed);
        g_run.rotate(right, 1);
        g_run.accelerate(HALF_SECTION, cruise_speed);
        break;
      case left:
        g_run.decelerate(HALF_SECTION + start_short, cruise_speed);
        g_run.rotate(left, 1);
        g_run.accelerate(HALF_SECTION, cruise_speed);
        break;
      case rear:
        g_run.decelerate(HALF_SECTION + start_short, cruise_speed);
        g_run.rotate(right, 2);
        g_run.accelerate(HALF_SECTION, cruise_speed);
        break;
      case loca_dir_error:
        // motorDisable()はnextDirGet()内で実行済み。詳細は§2.17参照
        return;
    }

    start_short = 0;
    g_map.mypos.dir = glob_nextdir;  //方向を更新
    g_map.axisUpdate();
  }

  g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
  ledWallShow(g_sensor.sen_l.is_wall, g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall);
  g_run.decelerate(HALF_SECTION + start_short, cruise_speed);
}

// adachiDynamic()と同じ骨格で、90度旋回を区画境界からのslalomCell()に置き換えた版(停止せず曲がる)。
// 行き止まりはuTurn()で回頭する。詳細はスラローム機能_実装まとめ.md §2.25・§2.65参照。
void SEARCH::adachiSlalomDynamic(char gx, char gy, char gx2, char gy2)
{
  t_global_direction glob_nextdir;

  // 現在位置から最初の進行方向へ向きを合わせる
  switch (g_map.nextDirGet(gx, gy, gx2, gy2, &glob_nextdir)) {
    case front:
      break;
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
      // 壁情報の破損等でゴール到達不能と判定(motorDisable()はnextDirGet()内で実行済み)。
      // 詳細はスラローム機能_実装まとめ.md §2.17参照
      return;
  }

  // 後ろ壁に合わせるのはスタート区画から出るときだけ(復路の開始は後ろに壁がない)
  g_run.startAccelerate(
    SECTION / 3, g_run.search_speed, (g_map.mypos.x == 0) && (g_map.mypos.y == 0));
  // 助走を短くした分を最初の距離基準の動作に足して区画境界へ合わせる(§2.47)
  int start_short = HALF_SECTION - SECTION / 3;

  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  // センサで壁情報を更新し、次の方向へ進む
  while (!g_map.posIn(gx, gy, gx2, gy2)) {
    // 側方壁はモード9専用のth_wall_slalomで判定する(§2.28)
    const bool is_wall_r = g_sensor.sen_r.value > g_sensor.sen_r.th_wall_slalom;
    const bool is_wall_l = g_sensor.sen_l.value > g_sensor.sen_l.th_wall_slalom;
    g_map.wallSet(g_sensor.sen_fr.is_wall, is_wall_r, is_wall_l);
    ledWallShow(is_wall_l, g_sensor.sen_fr.is_wall, is_wall_r);

    const t_local_direction local_dir = g_map.nextDirGet(gx, gy, gx2, gy2, &glob_nextdir);
    switch (local_dir) {
      case front:
        g_run.oneStepDynamic(
          SECTION + start_short, g_run.search_speed, g_sensor.th_approach_sum_slalom);
        break;
      case right:
      case left:
        if (start_short > 0) g_run.oneStep(start_short, (int)g_run.speed);
        g_run.slalomCell(local_dir);
        break;
      case rear:
        // 当てる壁のある側へ回る(§2.29・§2.70)
        g_run.decelerate(HALF_SECTION + start_short, g_run.speed);
        turnBack(g_sensor.sen_fr.is_wall, is_wall_r, is_wall_l);
        g_run.accelerate(HALF_SECTION, g_run.search_speed);
        break;
      case loca_dir_error:
        // motorDisable()はnextDirGet()内で実行済み。詳細は§2.17参照
        return;
    }

    start_short = 0;
    g_map.mypos.dir = glob_nextdir;  //方向を更新
    g_map.axisUpdate();
  }

  const bool goal_wall_r = g_sensor.sen_r.value > g_sensor.sen_r.th_wall_slalom;
  const bool goal_wall_l = g_sensor.sen_l.value > g_sensor.sen_l.th_wall_slalom;
  g_map.wallSet(g_sensor.sen_fr.is_wall, goal_wall_r, goal_wall_l);
  ledWallShow(goal_wall_l, g_sensor.sen_fr.is_wall, goal_wall_r);
  g_run.decelerate(HALF_SECTION + start_short, g_run.speed);
}
