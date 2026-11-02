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

// 左手法で壁を選びながら迷路を探索
void SEARCH::lefthand(void)
{
  g_run.startAccelerate(HALF_SECTION, g_run.search_speed);

  // スイッチが押されるまで探索走行を続ける
  // (区画の境界ごとに判定するため、実際に止まるのは押した次の区画)
  while (switchGet() == 0) {
    if (g_sensor.sen_l.is_wall == false) {
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(left, 1);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
    } else if ((g_sensor.sen_fl.is_wall == false) && (g_sensor.sen_fr.is_wall == false)) {
      g_run.straight(SECTION, g_run.search_speed, g_run.search_speed, g_run.search_speed);
    } else if (g_sensor.sen_r.is_wall == false) {
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(right, 1);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
    } else {
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(right, 2);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
    }
  }

  // スイッチが押されたら半区画で減速停止し、呼び出し元(modeExec)経由でモード選択へ戻る
  g_run.decelerate(HALF_SECTION, g_run.search_speed);
}

// 左手法で走行しながら通った区画の壁情報をg_mapに記録する。ゴール区画到達かスイッチ押下で終了。
// myposは呼び出し前にg_map.positionInit()で初期化しておくこと
void SEARCH::lefthandGetWall(char gx, char gy, char gx2, char gy2)
{
  // スタート区画から北へ半区画進み、最初の区画に入る
  g_run.startAccelerate(HALF_SECTION, g_run.search_speed);
  g_map.axisUpdate();

  while (1) {
    // 現在区画の壁情報を記録する
    g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);

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
    } else if ((g_sensor.sen_fl.is_wall == false) && (g_sensor.sen_fr.is_wall == false)) {
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

// 足立法で壁情報を更新しながらゴールまで探索
void SEARCH::adachi(char gx, char gy, char gx2, char gy2)
{
  t_global_direction glob_nextdir;
  t_local_direction temp_next_dir;

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
    HALF_SECTION, g_run.search_speed, (g_map.mypos.x == 0) && (g_map.mypos.y == 0));

  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  // センサで壁情報を更新し、次の方向へ進む
  while (!g_map.posIn(gx, gy, gx2, gy2)) {
    g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);

    switch (g_map.nextDirGet(gx, gy, gx2, gy2, &glob_nextdir)) {
      case front:
        g_run.oneStep(SECTION, g_run.search_speed);
        break;
      case right:
        g_run.decelerate(HALF_SECTION, g_run.search_speed);
        g_run.rotate(right, 1);
        g_run.accelerate(HALF_SECTION, g_run.search_speed);
        break;
      case left:
        g_run.decelerate(HALF_SECTION, g_run.search_speed);
        g_run.rotate(left, 1);
        g_run.accelerate(HALF_SECTION, g_run.search_speed);
        break;
      case rear:
        g_run.decelerate(HALF_SECTION, g_run.search_speed);
        g_run.rotate(right, 2);
        g_run.accelerate(HALF_SECTION, g_run.search_speed);
        break;
      case loca_dir_error:
        // motorDisable()はnextDirGet()内で実行済み。詳細は§2.17参照
        return;
    }

    g_map.mypos.dir = glob_nextdir;  //方向を更新
    g_map.axisUpdate();
  }

  g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
  g_run.decelerate(HALF_SECTION, g_run.search_speed);
}

// 左手法で迷路を探索(スラローム版)。lefthand()との違いは行き止まりのuTurn()のみで、90度旋回は
// rotate()のまま(slalom()化は壁衝突が頻発したため撤回、スラローム機能_実装まとめ.md §2.16参照)
void SEARCH::lefthandSlalom(void)
{
  g_run.startAccelerate(SECTION / 3, g_run.search_speed);

  // スイッチが押されるまで探索走行を続ける
  // (区画の境界ごとに判定するため、実際に止まるのは押した次の区画)
  while (switchGet() == 0) {
    if (g_sensor.sen_l.is_wall == false) {
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(left, 1);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
    } else if ((g_sensor.sen_fl.is_wall == false) && (g_sensor.sen_fr.is_wall == false)) {
      g_run.straight(SECTION, g_run.search_speed, g_run.search_speed, g_run.search_speed);
    } else if (g_sensor.sen_r.is_wall == false) {
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_run.rotate(right, 1);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
    } else {
      // 行き止まり: 90度旋回->バック->前進を2回行い、180度回頭しつつ縦横の位置を
      // 中心へ補正するuTurn()を使う(本フォークで追加)
      g_run.decelerate(SECTION / 3, g_run.search_speed);
      g_run.uTurn(right);
      g_run.accelerate(SECTION / 3, g_run.search_speed);
    }
  }

  // スイッチが押されたら半区画で減速停止し、呼び出し元(modeExec)経由でモード選択へ戻る
  g_run.decelerate(HALF_SECTION, g_run.search_speed);
}

// 足立法で壁情報を更新しながらゴールまで探索(スラローム版)。90度旋回はslalomCell()(§2.65)、
// 開始直後の方向合わせはrotate()、行き止まりはuTurn()
void SEARCH::adachiSlalom(char gx, char gy, char gx2, char gy2)
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
    HALF_SECTION, g_run.search_speed, (g_map.mypos.x == 0) && (g_map.mypos.y == 0));

  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  // センサで壁情報を更新し、次の方向へ進む
  while (!g_map.posIn(gx, gy, gx2, gy2)) {
    g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);

    const t_local_direction local_dir = g_map.nextDirGet(gx, gy, gx2, gy2, &glob_nextdir);
    switch (local_dir) {
      case front:
        g_run.oneStep(SECTION, g_run.search_speed);
        break;
      case right:
      case left:
        g_run.slalomCell(local_dir);
        break;
      case loca_dir_error:
        // motorDisable()はnextDirGet()内で実行済み。詳細は§2.17参照
        return;
      case rear:
        // 当てる壁があればuTurn()で位置を補正しながら回頭する(§2.70)
        g_run.decelerate(HALF_SECTION, g_run.speed);
        turnBack(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
        g_run.accelerate(HALF_SECTION, g_run.search_speed);
        break;
    }

    g_map.mypos.dir = glob_nextdir;  //方向を更新
    g_map.axisUpdate();
  }

  g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
  g_run.decelerate(HALF_SECTION, g_run.speed);
}