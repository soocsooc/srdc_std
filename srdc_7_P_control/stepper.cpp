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
#include "stepper.h"

#define STEP_TIMER_HZ 2000000  // パルス周期を数えるタイマーの分解能(0.5us)
#define STEP_HZ_MIN 30         // 周期の上限(約33ms)。速度の変更は次のパルスで反映される

static hw_timer_t * g_timer2 = NULL;  // 右モータ
static hw_timer_t * g_timer3 = NULL;  // 左モータ
static portMUX_TYPE g_stepper_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_motor_move = false;
static volatile unsigned short g_step_hz_r = STEP_HZ_MIN;
static volatile unsigned short g_step_hz_l = STEP_HZ_MIN;
static volatile int g_step_r = 0, g_step_l = 0;          // 前進方向を正とするパルス数
static volatile int g_step_sign_r = 1, g_step_sign_l = 1;  // +1:前進, -1:後退

// CLOCKピンに1パルス出す
static inline void IRAM_ATTR clockPulse(uint8_t pin)
{
  digitalWrite(pin, HIGH);
  for (int i = 0; i < 100; i++) {
    asm("nop \n");
  }
  digitalWrite(pin, LOW);
}

// Rモータの周期数割り込み。次の割り込みまでの時間を、現在の周波数から設定し直す
static void IRAM_ATTR isrR(void)
{
  portENTER_CRITICAL_ISR(&g_stepper_mux);
  if (g_motor_move) {
    timerAlarm(g_timer2, STEP_TIMER_HZ / g_step_hz_r, true, 0);
    clockPulse(CLOCK_R);
    g_step_r += g_step_sign_r;
  }
  portEXIT_CRITICAL_ISR(&g_stepper_mux);
}

// Lモータの周期数割り込み
static void IRAM_ATTR isrL(void)
{
  portENTER_CRITICAL_ISR(&g_stepper_mux);
  if (g_motor_move) {
    timerAlarm(g_timer3, STEP_TIMER_HZ / g_step_hz_l, true, 0);
    clockPulse(CLOCK_L);
    g_step_l += g_step_sign_l;
  }
  portEXIT_CRITICAL_ISR(&g_stepper_mux);
}

void stepperInit(void)
{
  pinMode(CW_R, OUTPUT);
  pinMode(CW_L, OUTPUT);
  pinMode(CLOCK_R, OUTPUT);
  pinMode(CLOCK_L, OUTPUT);
  digitalWrite(CLOCK_R, LOW);
  digitalWrite(CLOCK_L, LOW);
  stepperDirSet(true, true);

  g_timer2 = timerBegin(STEP_TIMER_HZ);  //2MHz(0.5us)
  timerAttachInterrupt(g_timer2, &isrR);
  timerAlarm(g_timer2, STEP_TIMER_HZ / STEP_HZ_MIN, true, 0);
  timerStart(g_timer2);

  g_timer3 = timerBegin(STEP_TIMER_HZ);
  timerAttachInterrupt(g_timer3, &isrL);
  timerAlarm(g_timer3, STEP_TIMER_HZ / STEP_HZ_MIN, true, 0);
  timerStart(g_timer3);
}

void stepperDirSet(bool forward_l, bool forward_r)
{
  portENTER_CRITICAL(&g_stepper_mux);
  digitalWrite(CW_L, forward_l ? CW_L_FORWARD : !CW_L_FORWARD);
  digitalWrite(CW_R, forward_r ? CW_R_FORWARD : !CW_R_FORWARD);
  g_step_sign_l = forward_l ? 1 : -1;
  g_step_sign_r = forward_r ? 1 : -1;
  portEXIT_CRITICAL(&g_stepper_mux);
}

static unsigned short stepHzClamp(double hz)
{
  hz = fabs(hz);
  if (hz < STEP_HZ_MIN) return STEP_HZ_MIN;
  if (hz > 65535.0) return 65535;
  return (unsigned short)hz;
}

void stepperHzSet(double hz_l, double hz_r)
{
  g_step_hz_l = stepHzClamp(hz_l);
  g_step_hz_r = stepHzClamp(hz_r);
  g_motor_move = true;
}

void stepperStop(void) { g_motor_move = false; }

void stepperCounterClear(void)
{
  portENTER_CRITICAL(&g_stepper_mux);
  g_step_l = 0;
  g_step_r = 0;
  portEXIT_CRITICAL(&g_stepper_mux);
}

void stepperCounterGet(int * l, int * r)
{
  portENTER_CRITICAL(&g_stepper_mux);
  *l = g_step_l;
  *r = g_step_r;
  portEXIT_CRITICAL(&g_stepper_mux);
}
