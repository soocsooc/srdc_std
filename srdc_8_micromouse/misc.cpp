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

MISC g_misc;

// ボタン入力値を増加させ、上限を越えたら先頭へ戻す
short MISC::buttonInc(short data, short limit, short limit_data)
{
  data++;
  if (data > limit) {
    data = limit_data;
  }
  buzzerEnable(INC_FREQ);
  delay(30);
  buzzerDisable();
  return data;
}

// 決定ボタンの確認音を鳴らす
void MISC::buttonOk(void)
{
  buzzerEnable(DEC_FREQ);
  delay(80);
  buzzerEnable(INC_FREQ);
  delay(80);
  buzzerDisable();
}

// ゴール到達をLEDで通知し、探索した地図を保存
void MISC::goalAppeal(void)  //ゴールしたことをアピールする
{
  unsigned char led_data;

  int wtime = 100;

  motorDisable();
  mapWrite();

  for (int j = 0; j < 10; j++) {
    led_data = 1;
    for (int i = 0; i < 4; i++) {
      ledSet(led_data);
      led_data <<= 1;
      delay(wtime);
    }
    led_data = 8;
    for (int i = 0; i < 4; i++) {
      ledSet(led_data);
      led_data >>= 1;
      delay(wtime);
    }
    wtime -= 5;
  }

  delay(500);
  motorEnable();
}

// プリセットを選ぶ。LED4の点滅中、LED1〜3の点灯位置が番号を示す(SW1で変更、SW2で決定)
unsigned char MISC::presetSelect(void)
{
  unsigned char sel = (preset_select < PRESET_NUM) ? preset_select : 0;
  bool blink = false;

  while (1) {
    const unsigned char sw = switchGet();
    if (sw == SW_RM) {
      sel = buttonInc(sel, PRESET_NUM - 1, 0);
    } else if (sw == SW_CM) {
      buttonOk();
      return sel;
    }
    blink = !blink;
    ledSet((1 << sel) | (blink ? 0b1000 : 0));
    delay(100);
  }
}

// 走行モードはプリセットを選び、速度・加速度をベース値×%にして実行する。終了後にベース値へ戻すので、
// モード1・2のようにmodeExec()が入れ子になっても二重に掛からない(設計メモ §12)
void MISC::modeRun(int mode)
{
  if ((mode < 1) || (mode > 9)) {
    modeExec(mode);
    return;
  }

  preset_select = presetSelect();
  const long sp = preset_speed_pct[preset_select];
  const long ap = preset_accel_pct[preset_select];

  const short base_search_speed = g_run.search_speed;
  const short base_max_speed = g_run.max_speed;
  const short base_slalom_speed = g_run.slalom_speed;
  const float base_search_accel = g_run.search_accel;
  const float base_turn_accel = g_run.turn_accel;

  g_run.search_speed = (short)constrain(base_search_speed * sp / 100, MIN_SPEED, SPEED_SANITY_MAX);
  g_run.max_speed = (short)constrain(base_max_speed * sp / 100, MIN_SPEED, SPEED_SANITY_MAX);
  g_run.slalom_speed = (short)constrain(base_slalom_speed * sp / 100, MIN_SPEED, SPEED_SANITY_MAX);
  g_run.search_accel = base_search_accel * ap / 100.0f;
  g_run.turn_accel = base_turn_accel * ap / 100.0f;
  Serial.printf("preset %d: speed %ld%% accel %ld%%\r\n", preset_select + 1, sp, ap);

  modeExec(mode);

  g_run.search_speed = base_search_speed;
  g_run.max_speed = base_max_speed;
  g_run.slalom_speed = base_slalom_speed;
  g_run.search_accel = base_search_accel;
  g_run.turn_accel = base_turn_accel;
}

// modeExec()のcaseと対応させること
void MISC::modeListPrint(void)
{
  static const char * const names[] = {
    "1: lefthand",
    "2: adachi search",
    "3: fast run",
    "4: lefthand (get wall)",
    "5: lefthand slalom",
    "6: adachi search slalom",
    "7: fast run slalom",
    "8: adachi search dynamic",
    "9: adachi search dynamic slalom",
    "10: erase map",
    "15: adjust menu",
  };
  for (const char * n : names) Serial.println(n);
}

// 選択された走行・調整モードを実行
void MISC::modeExec(int mode)
{
  // 実行するモードを選択状態として記録する(最短走行3/7を含め全モード共通、設計メモ §13参照)
  mode_select = mode;
  g_map.unreachable = false;
  motorEnable();
  delay(1000);

  switch (mode) {
    case 1:
      g_search.lefthand();
      break;
    case 2:  //足立法
      g_map.positionInit();
      g_search.adachi(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);	// goalへ向かう
      if (g_map.unreachable) break;  // 到達不能になりスイッチで脱出したら中断(設計メモ §8)
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachi(0, 0, 0, 0);	// スタートに戻る
      if (g_map.unreachable) break;
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      break;
    case 3:  //最短走行
      mapCopy();
      g_map.positionInit();
      g_fast.run(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);
      if (g_map.unreachable) break;  // 到達不能になりスイッチで脱出したら中断(設計メモ §8)
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachiDynamic(0, 0, 0, 0);  // 復路は未知区画を通る探索で戻る(設計メモ §11)
      if (g_map.unreachable) break;
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      break;
    case 4:  //迷路情報を取得する左手法
      g_map.positionInit();
      g_search.lefthandGetWall(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);
      mapWrite();
      break;
    case 5:  //左手法(スラローム版)
      g_search.lefthandSlalom();
      break;
    case 6:  //足立法(スラローム版)
      g_map.positionInit();
      g_search.adachiSlalom(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);	// goalへ向かう
      if (g_map.unreachable) break;  // 到達不能になりスイッチで脱出したら中断(設計メモ §8)
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachiSlalom(0, 0, 0, 0);	// スタートに戻る
      if (g_map.unreachable) break;
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      break;
    case 7:  //最短走行(スラローム版)
      mapCopy();
      g_map.positionInit();
      g_fast.runSlalom(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);
      if (g_map.unreachable) break;  // 到達不能になりスイッチで脱出したら中断(設計メモ §8)
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachiSlalomDynamic(0, 0, 0, 0);  // 復路はモード9の探索で戻る(設計メモ §11)
      if (g_map.unreachable) break;
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      break;
    case 8:  //足立法(センサ閾値主導の動的到達判定版)
      g_map.positionInit();
      g_search.adachiDynamic(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);	// goalへ向かう
      if (g_map.unreachable) break;  // 到達不能になりスイッチで脱出したら中断(設計メモ §8)
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachiDynamic(0, 0, 0, 0);	// スタートに戻る
      if (g_map.unreachable) break;
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      break;
    case 9:  //足立法(センサ閾値主導の動的到達判定版・スラローム版)
      g_map.positionInit();
      g_search.adachiSlalomDynamic(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);	// goalへ向かう
      if (g_map.unreachable) break;  // 到達不能になりスイッチで脱出したら中断(設計メモ §8)
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachiSlalomDynamic(0, 0, 0, 0);	// スタートに戻る
      if (g_map.unreachable) break;
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      break;
    case 10:  //Flash上のマップ情報を消去
      mapErase();
      g_misc.buttonOk();  //完了を音で通知
      break;
    case 11:
      break;
    case 12:
      break;
    case 13:
      break;
    case 14:
      break;
    case 15:
      motorDisable();
      g_adjust.menu();  //調整メニューに行く
      break;
    default:
      break;
  }

  ledSet(mode);  // 走行中のLED表示を、実行したモード番号の表示へ戻す(設計メモ §13)
  motorDisable();
}
