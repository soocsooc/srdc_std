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

SENSOR g_sensor;

// sensorTask(device.cpp)から1ms周期で4回呼ばれ、SENSOR::interrupt()で電池電圧・前・右・左を
// 1つずつ処理する
void sensorInterrupt(void) { g_sensor.interrupt(); }

static short senScale(short v, short pct) { return (short)(((long)v * pct + 50) / 100); }

// REF・しきい値のベース値に倍率を掛けて判定用の値にする(設計メモ §12)
void SENSOR::applyRatio(void)
{
  if ((ratio_pct < SEN_RATIO_MIN) || (ratio_pct > SEN_RATIO_MAX)) ratio_pct = SEN_RATIO_INIT;
  const short p = ratio_pct;
  sen_r.ref = senScale(base.ref_r, p);
  sen_l.ref = senScale(base.ref_l, p);
  sen_r.th_wall = senScale(base.th_r, p);
  sen_l.th_wall = senScale(base.th_l, p);
  sen_f.th_wall = senScale(base.th_f, p);
  sen_r.th_wall_slalom = senScale(base.th_r_slalom, p);
  sen_l.th_wall_slalom = senScale(base.th_l_slalom, p);
  th_approach = senScale(base.th_approach, p);
  th_approach_slalom = senScale(base.th_approach_slalom, p);
}

// --- センサ値の平滑化 ------------------------------------------------------
// 1パルス1リード(捨て読みは信号を潰すため禁止)で、senMedian3(スパイク除去)->senFilter(IIR、
// 時定数≒3ms)の2段で処理する。ch: 0=前 1=右 2=左

static short g_raw_hist[3][3];
static unsigned char g_raw_pos[3];
static bool g_raw_full[3];

static short senMedian3(unsigned char ch, unsigned short raw)
{
  g_raw_hist[ch][g_raw_pos[ch]] = (short)raw;
  if (++g_raw_pos[ch] >= 3) {
    g_raw_pos[ch] = 0;
    g_raw_full[ch] = true;
  }
  if (!g_raw_full[ch]) return (short)raw;  // 3個揃うまでは生値

  short a = g_raw_hist[ch][0], b = g_raw_hist[ch][1], c = g_raw_hist[ch][2];
  short t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}

static inline short senFilter(short prev, short in)
{
  if (prev == 0) return in;  // 起動直後は即収束
  return (short)(((int)prev * SEN_IIR_A + (int)in) / (SEN_IIR_A + 1));
}

// 生読み値 -> メディアン -> IIR。SENSOR_SAMPLE_BADはメディアン履歴にもIIRにも入れず前回値を維持する
static inline short senProcess(unsigned char ch, short prev, unsigned short raw)
{
  if (raw == SENSOR_SAMPLE_BAD) {
    g_sensor.sample_reject[ch]++;
    return prev;
  }
  return senFilter(prev, senMedian3(ch, raw));
}

// センサを順番に読み取り、壁判定と壁制御用誤差を更新する。起床直後のジッタを避けるため、
// SLEDを使う読み取りはcnt==1以降に置くこと(STEP8_動作まとめ.md §7参照)
void SENSOR::interrupt(void)
{
  static char cnt = 0;
  static unsigned short batt_low_cnt = 0;  // 電池低下の連続カウント[ms]

  // 4回の呼び出しで電池電圧処理+前・右・左のセンサを1巡させる
  switch (cnt) {
    case 0:
      battery_value = batteryVoltGet();
      // WiFi TX 電流バーストでレールが一瞬落ちると電池ADCも一時的に下がる。
      // 単発では鳴らさず、BATT_LOW_ALARM_MS ミリ秒連続で低下したときだけ警報。
      if (battery_value < BATT_MIN) {
        if (batt_low_cnt < BATT_LOW_ALARM_MS) batt_low_cnt++;
      } else {
        batt_low_cnt = 0;
      }
      if (batt_low_cnt >= BATT_LOW_ALARM_MS) {
        buzzerEnable(400);
        ledSet(0);
      }
      break;
    case 1:
      sen_f.value = senProcess(0, sen_f.value, sensorGetF());
      sen_f.is_wall = (sen_f.value > sen_f.th_wall);
      break;
    case 2:
      sen_r.value = senProcess(1, sen_r.value, sensorGetR());
      // 壁ありのときだけP制御に使う
      if (sen_r.value > sen_r.th_wall) {
        sen_r.is_wall = true;
        sen_r.error = sen_r.value - sen_r.ref;
        sen_r.is_control = true;
      } else {
        sen_r.is_wall = false;
        sen_r.error = 0;
        sen_r.is_control = false;
      }
      break;
    case 3:
      sen_l.value = senProcess(2, sen_l.value, sensorGetL());
      if (sen_l.value > sen_l.th_wall) {
        sen_l.is_wall = true;
        sen_l.error = sen_l.value - sen_l.ref;
        sen_l.is_control = true;
      } else {
        sen_l.is_wall = false;
        sen_l.error = 0;
        sen_l.is_control = false;
      }
      break;
    default:
      Serial.printf("sensor state error\n\r");
      break;
  }
  cnt++;
  if (cnt >= 4) cnt = 0;
}
