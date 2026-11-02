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

// LEDはLOWで点灯する
#define LED0 5
#define LED1 13
#define LED2 15
#define LED3 16

// SW1は内部プルアップを使う。SW2(GPIO35)は内部プルアップがないため、10kの外付けプルアップが必要
#define SW1 33
#define SW2 35

int g_state_1, g_state_2;

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT);

  g_state_1 = g_state_2 = 0;
}

void loop()
{
  // いずれかのSWが押される(=LOW)まで待機
  while (digitalRead(SW1) && digitalRead(SW2)) {
    continue;
  }

  // SWに応じたLEDを点灯・消灯 (stateをインクリメントし、奇数のとき点灯=LOW)
  if (digitalRead(SW1) == 0) {
    ++g_state_1;
    digitalWrite(LED0, (g_state_1 & 0x01) ? LOW : HIGH);
    digitalWrite(LED1, (g_state_1 & 0x01) ? LOW : HIGH);
  }
  if (digitalRead(SW2) == 0) {
    ++g_state_2;
    digitalWrite(LED2, (g_state_2 & 0x01) ? LOW : HIGH);
    digitalWrite(LED3, (g_state_2 & 0x01) ? LOW : HIGH);
  }

  delay(30);  // チャタリング回避

  // SWが離れるまで待機
  while (!(digitalRead(SW1) && digitalRead(SW2))) {
    continue;
  }

  delay(30);  // チャタリング回避
}
