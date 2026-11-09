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

// センサLED(HIGHで点灯)。前は1個
#define SLED_F 14
#define SLED_R 4
#define SLED_L 27

// 受光素子(ADC1)。前・右・左の対応は仮のため、実機で確認すること
#define AD_F 36
#define AD_R 39
#define AD_L 34
#define AD_BATT 32

volatile short f_sen;
volatile short r_sen;
volatile short l_sen;
volatile short vdd;

hw_timer_t * g_timer1 = NULL;
portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer1(void);

void setup()
{
  // センサLEDを消灯しておく
  pinMode(SLED_F, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_F, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  Serial.begin(115200);

  g_timer1 = timerBegin(1000000);  // 1MHz(1us)で数えるタイマー
  timerAttachInterrupt(g_timer1, &onTimer1);
  timerAlarm(g_timer1, 250, true, 0);  // 250 * 1us = 250us(4kHz)ごとに割り込む
  timerStart(g_timer1);
}

void loop()
{
  Serial.printf(">r_sen:%d\n", r_sen);
  Serial.printf(">f_sen:%d\n", f_sen);
  Serial.printf(">l_sen:%d\n", l_sen);
  Serial.printf(">vdd:%d\n", vdd);
  delay(100);
}

// 4kHzのタイマ割り込み。呼ばれるたびに前・右・左のセンサを1つずつ読み、左の回で電池電圧も測る
void IRAM_ATTR onTimer1(void)
{
  static char cnt = 0;
  portENTER_CRITICAL_ISR(&g_timer_mux);  // 他からの割り込みの禁止
  switch (cnt) {                         // センサを順番に読む
    case 0:
      digitalWrite(SLED_F, HIGH);  // LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");  // 受光の出力が出るまで待つ
      }
      f_sen = analogRead(AD_F);
      digitalWrite(SLED_F, LOW);  // LED消灯
      break;
    case 1:
      digitalWrite(SLED_R, HIGH);  // LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      r_sen = analogRead(AD_R);
      digitalWrite(SLED_R, LOW);  // LED消灯
      break;
    case 2:
      digitalWrite(SLED_L, HIGH);  // LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      l_sen = analogRead(AD_L);
      digitalWrite(SLED_L, LOW);  // LED消灯
      vdd = (double)analogReadMilliVolts(AD_BATT) / 10.0 *
            (10.0 + 51.0);  // 電池電圧[mV]。51kと10kの分圧から計算する
      break;
    default:
      Serial.printf("error\n\r");
      break;
  }
  cnt++;
  if (cnt >= 3) cnt = 0;
  portEXIT_CRITICAL_ISR(&g_timer_mux);  // 割り込み禁止の解除
}
