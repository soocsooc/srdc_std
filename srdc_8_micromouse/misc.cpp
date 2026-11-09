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
void MISC::goalAppeal(void)
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

// 走行モードの一覧をシリアルへ出力する(USBシリアルのlist_r)。modeExec()のcaseと対応させること
void MISC::modeListPrint(void)
{
  static const char * const names[] = {
    "1: lefthand (get wall)",
    "2: adachi search",
    "3: adachi search slalom",
    "4: fast run",
    "5: fast run slalom",
    "6: erase map",
    "7: adjust menu",
  };
  for (const char * n : names) Serial.println(n);
}

// 選択された走行・調整モードを実行
void MISC::modeExec(int mode)
{
  // 実行するモードを選択状態として記録する(全モード共通、設計メモ §13参照)
  mode_select = mode;
  g_map.unreachable = false;
  motorEnable();
  delay(1000);

  switch (mode) {
    case 1:  //迷路情報を取得する左手法
      g_map.positionInit();
      g_search.lefthandGetWall(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);
      mapWrite();
      break;
    case 2:  //足立法(前壁で区画境界を判定)
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
    case 3:  //足立法(前壁で区画境界を判定、スラローム版)
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
    case 4:  //最短走行
      mapCopy();
      g_map.positionInit();
      g_fast.run(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);
      if (g_map.unreachable) break;  // 到達不能になりスイッチで脱出したら中断(設計メモ §8)
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachiDynamic(0, 0, 0, 0);  // 復路はモード2の探索で戻る(設計メモ §11)
      if (g_map.unreachable) break;
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      break;
    case 5:  //最短走行(スラローム版)
      mapCopy();
      g_map.positionInit();
      g_fast.runSlalom(g_map.goal_mx, g_map.goal_my, g_map.goal_mx2, g_map.goal_my2);
      if (g_map.unreachable) break;  // 到達不能になりスイッチで脱出したら中断(設計メモ §8)
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachiSlalomDynamic(0, 0, 0, 0);  // 復路はモード3の探索で戻る(設計メモ §11)
      if (g_map.unreachable) break;
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      break;
    case MODE_ERASE_MAP:  //Flash上のマップ情報を消去
      mapErase();
      g_misc.buttonOk();  //完了を音で通知
      break;
    case MODE_ADJUST:
      motorDisable();
      g_adjust.menu();  //調整メニューに行く
      break;
    default:
      break;
  }

  ledSet(mode);  // 走行中のLED表示を、実行したモード番号の表示へ戻す(設計メモ §13)
  motorDisable();
}
