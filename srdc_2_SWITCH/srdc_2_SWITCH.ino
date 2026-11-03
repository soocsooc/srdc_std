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

// LEDはHIGHで点灯する。LED0〜3は回路図のL_1〜L_4
#define LED0 5
#define LED1 13
#define LED2 16
#define LED3 15

// SW1は内部プルアップを使う。SW2(GPIO35)は内部プルアップがなく、基板にもプルアップがない。
// 基板を直して3.3Vへの10kのプルアップを付けるまではUSE_SW2を0にしておき、SW2を読まない
#define SW1 33
#define SW2 35
#define USE_SW2 0

int g_state_1, g_state_2;

// スイッチが押されている(=LOW)か。USE_SW2が0のあいだ、SW2は押されていないとみなす
bool sw1Pressed(void) { return digitalRead(SW1) == LOW; }
bool sw2Pressed(void) { return USE_SW2 && (digitalRead(SW2) == LOW); }

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT);

  g_state_1 = g_state_2 = 0;
}

void loop()
{
  // いずれかのSWが押されるまで待機
  while (!sw1Pressed() && !sw2Pressed()) {
    continue;
  }

  // SWに応じたLEDを点灯・消灯 (stateをインクリメントし、2進数最終bitの偶奇をLEDに出力)
  if (sw1Pressed()) {
    ++g_state_1;
    digitalWrite(LED0, g_state_1 & 0x01);
    digitalWrite(LED1, g_state_1 & 0x01);
  }
  if (sw2Pressed()) {
    ++g_state_2;
    digitalWrite(LED2, g_state_2 & 0x01);
    digitalWrite(LED3, g_state_2 & 0x01);
  }

  delay(30);  // チャタリング回避

  // SWが離れるまで待機
  while (sw1Pressed() || sw2Pressed()) {
    continue;
  }

  delay(30);  // チャタリング回避
}
