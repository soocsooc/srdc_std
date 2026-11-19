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
// LEDの1回の点灯で1回だけ読む(捨て読みを入れると信号が小さくなるため入れない)。senMedian3()で
// 突発的な値を除き、senFilter()(IIR、時定数約3ms)で平滑化する。ch: 0=前 1=右 2=左

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
  if (prev == 0) return in;  // 起動直後は読み取り値をそのまま使う
  return (short)(((int)prev * SEN_IIR_A + (int)in) / (SEN_IIR_A + 1));
}

// 読み取り値をメディアン、IIRの順に通す。SENSOR_SAMPLE_BADはどちらにも入れず前回値を使う
static inline short senProcess(unsigned char ch, short prev, unsigned short raw)
{
  if (raw == SENSOR_SAMPLE_BAD) {
    g_sensor.sample_reject[ch] = g_sensor.sample_reject[ch] + 1;
    return prev;
  }
  return senFilter(prev, senMedian3(ch, raw));
}

// 横センサの壁判定と壁制御用の誤差を更新する。壁ありのときだけP制御に使う。
// 柱を読むと値が急に下がるので、SEN_DROP_LOOKBACK[ms]前より基準値のSEN_DROP_PCT[%]を超えて
// 下がったら落ち込み中とし、その間は負の誤差(壁から遠い向き)を0にする。落ち込み中の最小値から
// 下がり幅のしきい値以上戻るか、下がる前の値近くまで戻るか、SEN_DIP_MAX_LEN[mm]走ったら終える。
// 柱を読む間に機体が壁から離れても、谷を過ぎた時点で終わる(設計メモ §14)
static void sideUpdate(volatile t_sensor & s)
{
  const short old = s.hist[s.hist_idx];  // SEN_DROP_LOOKBACK[ms]前の値
  s.hist[s.hist_idx] = s.value;
  if (++s.hist_idx >= SEN_DROP_LOOKBACK) s.hist_idx = 0;

  if (s.value <= s.th_wall) {
    s.is_wall = false;
    s.error = 0;
    s.is_control = false;
    s.is_dip = false;
    return;
  }

  const short drop_th = (short)((long)s.ref * SEN_DROP_PCT / 100);
  if (!s.is_dip) {
    if ((old - s.value) > drop_th) {
      s.is_dip = true;
      s.dip_base = old;
      s.dip_min = s.value;
      s.dip_len = 0.0f;
    }
  } else {
    if (s.value < s.dip_min) s.dip_min = s.value;
    s.dip_len += (float)g_run.speed * 0.001f;  // 1msで進んだ距離[mm]
    if (
      (s.value >= s.dip_min + drop_th) || (s.value >= s.dip_base - drop_th / 2) ||
      (s.dip_len > SEN_DIP_MAX_LEN)) {
      s.is_dip = false;
    }
  }

  s.is_wall = true;
  s.error = s.value - s.ref;
  if (s.is_dip && (s.error < 0)) s.error = 0;
  s.is_control = true;
}

// センサを順番に読み取り、壁判定と壁制御用誤差を更新する。タスクが起きた直後は読むタイミングが
// 揺れるため、SLEDを使う読み取りはcnt==1以降に置く(STEP8_GUIDE.md §7)
void SENSOR::interrupt(void)
{
  static char cnt = 0;
  static unsigned short batt_low_cnt = 0;  // 電池低下の連続カウント[ms]

  // 4回の呼び出しで電池電圧処理+前・右・左のセンサを1巡させる
  switch (cnt) {
    case 0:
      battery_value = batteryVoltGet();
      // WiFiの送信で電源電圧が一瞬下がると、電池電圧の読み取り値も下がる。
      // そのため、BATT_LOW_ALARM_MS[ms]続けて下回ったときだけブザーを鳴らす
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
      sideUpdate(sen_r);
      break;
    case 3:
      sen_l.value = senProcess(2, sen_l.value, sensorGetL());
      sideUpdate(sen_l);
      break;
    default:
      Serial.printf("sensor state error\n\r");
      break;
  }
  cnt++;
  if (cnt >= 4) cnt = 0;
}
