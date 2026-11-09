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

#include <Arduino.h>
#include "pins_config.h"
#include "sensor.h"

// SENSORクラスのインスタンスおよびメンバ関数の定義
SENSOR g_sensor;

// SENSORクラスのコンストラクタの定義
SENSOR::SENSOR()
{
  sen_r.ref = REF_SEN_R;
  sen_l.ref = REF_SEN_L;

  sen_r.th_wall = TH_SEN_R;
  sen_l.th_wall = TH_SEN_L;

  sen_f.th_wall = TH_SEN_F;
}

// センサ用のタイマ割り込み(onTimer1)から呼ばれ、SENSOR::interrupt()でセンサを1つ処理する
void sensorInterrupt(void) { g_sensor.interrupt(); }

// 呼ばれるたびに前・右・左のセンサを1つずつ読み、壁の有無と壁制御用の誤差を更新する。
// 左の回で電池電圧も測る
void SENSOR::interrupt(void)
{
  static char cnt = 0;

  switch (cnt) {	// センサーの順番に応じた処理
    case 0:
      digitalWrite(SLED_F, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sen_f.value = analogRead(AD_F);
      digitalWrite(SLED_F, LOW);  //LED消灯
      sen_f.is_wall = (sen_f.value > sen_f.th_wall);  // 前壁検知
      break;
    case 1:
      digitalWrite(SLED_R, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sen_r.value = analogRead(AD_R);
      digitalWrite(SLED_R, LOW);  //LED消灯
      if (sen_r.value > sen_r.th_wall) {	// 右壁があればP制御にも使う
        sen_r.is_wall = true;
        sen_r.error = sen_r.value - sen_r.ref; // 右壁誤差 = センサー値 - 右壁基準値
        sen_r.is_control = true;
      } else {
        sen_r.is_wall = false;
        sen_r.error = 0;
        sen_r.is_control = false;
      }
      break;
    case 2:
      digitalWrite(SLED_L, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sen_l.value = analogRead(AD_L);
      digitalWrite(SLED_L, LOW);  //LED消灯
      if (sen_l.value > sen_l.th_wall) {	// 左壁があればP制御にも使う
        sen_l.is_wall = true;
        sen_l.error = sen_l.value - sen_l.ref; // 左壁誤差 = センサー値 - 左壁基準値
        sen_l.is_control = true;
      } else {
        sen_l.is_wall = false;
        sen_l.error = 0;
        sen_l.is_control = false;
      }
      battery_value = (double)analogReadMilliVolts(AD_BATT) / 10.0 * (10.0 + 51.0);	// バッテリー電圧計算
      break;
    default:
      Serial.printf("sensor state error\n\r");
      break;
  }
  cnt++;
  if (cnt >= 3) cnt = 0;
}
