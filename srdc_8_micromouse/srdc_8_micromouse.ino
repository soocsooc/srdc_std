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

void setup()
{
  // デバイス、フラッシュ、パラメータを初期化
  deviceInit();
  flashBegin();
  paramRead();
  mapCopy();  // 電源を切っても前回の地図から続けられるようにする(設計メモ §10)

  // 起動音を鳴らしてからモーターを無効化
  motorDisable();
  buzzerEnable(INC_FREQ);
  delay(80);
  buzzerDisable();

  // 起動時はモード選択画面から開始
  g_misc.mode_select = 1;
}

void loop()
{
  // 待機中はUSBシリアルからのパラメータ設定を受け付ける
  paramSerialPoll();
  // 選択中のモードをLEDで表示
  ledSet(g_misc.mode_select);
  switch (switchGet()) {
    case SW_RM:
      // 「次」(SW1の短押し)で次のモードへ移動(MODE_ADJUSTの次は1)
      g_misc.mode_select = g_misc.buttonInc(g_misc.mode_select, MODE_ADJUST, 1);
      break;
    case SW_CM:
      // 「決定」(SW2、USE_SW2が0ならSW1の長押し)で選択したモードを実行
      g_misc.buttonOk();
      g_misc.modeExec(g_misc.mode_select);
      break;
  }
  delay(1);
}
