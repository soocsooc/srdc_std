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

hw_timer_t * g_timer0 = NULL;

portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

// 1kHzの走行制御タイマー割り込み。controlInterrupt()で速度と壁制御を更新する
void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  controlInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void controlInterruptStart(void) { timerStart(g_timer0); }
void controlInterruptStop(void) { timerStop(g_timer0); }

// --- センサ計測タスク (core1固定) -------------------------------------------
// WiFi(core0)と分離し、起床直後の1回目はSLEDを使わない処理に充てる(sensor.cppのcnt==0)。
// 経緯はSTEP8_動作まとめ.md §7参照
#define SENSOR_TASK_CORE 1
#define SENSOR_TASK_PRIORITY 20  // loopTask(1)/AsyncTCP(<=10)より上、WiFi内部より下
#define SENSOR_TASK_STACK 4096

static TaskHandle_t g_sensor_task = NULL;
static volatile bool g_sensor_run = true;     // false でセンサ読みを一時停止
static volatile bool g_sensor_paused = false;  // タスクが停止点に到達したら true

static void sensorTask(void * arg)
{
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (g_sensor_run) {
      g_sensor_paused = false;
      // g_sensor.interrupt(): cnt 0(電池,SLEDなし)->3(左)で1巡(sensor.cpp参照)
      for (int i = 0; i < 4; i++) {
        sensorInterrupt();
      }
    } else {
      g_sensor_paused = true;
    }
    vTaskDelayUntil(&last, 1);  // 1ms 周期 (FreeRTOS tick = 1ms)
  }
}

static void stepperTimerStart(void);
static void stepperTimerStop(void);

void sensorInterruptStart(void)
{
  g_sensor_run = true;
  stepperTimerStart();
}

void sensorInterruptStop(void)
{
  // Flash 書き込み等の前に呼ばれる。進行中の 1 サイクル完了を待って停止する。
  // Flash書き込み中はキャッシュが止まるため、パルス生成のタイマーも止める(v3のPWMInterruptStop相当)
  g_sensor_run = false;
  for (int i = 0; i < 10 && !g_sensor_paused; i++) delay(1);
  stepperTimerStop();
}

// --- ステッピングモータのパルス生成 (EIC4311/SLA7078互換) ---------------------
// v3(Pi:Co Classic3)と同じく、左右のタイマー割り込みでCLOCKにパルスを出し、パルス数を数える。
// 周期は次のパルスで反映されるため、速度の変更は最大1パルス分遅れる
#define STEP_TIMER_HZ 2000000
#define STEP_HZ_MIN 30  // 周期の上限(約33ms)。これより遅いと速度の変更が反映されるまで待たされる

static hw_timer_t * g_timer2 = NULL;  // 右モータ
static hw_timer_t * g_timer3 = NULL;  // 左モータ
static volatile bool g_motor_move = false;
static volatile unsigned short g_step_hz_r = STEP_HZ_MIN;
static volatile unsigned short g_step_hz_l = STEP_HZ_MIN;
static volatile int g_step_r = 0, g_step_l = 0;          // 前進方向を正とする符号付きパルス数
static volatile int g_step_sign_r = 1, g_step_sign_l = 1;  // 現在の回転方向(+1:前進, -1:後退)

static inline void IRAM_ATTR clockPulse(uint8_t pin)
{
  digitalWrite(pin, HIGH);
  for (int i = 0; i < 100; i++) {
    asm("nop \n");
  }
  digitalWrite(pin, LOW);
}

void IRAM_ATTR isrR(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  if (g_motor_move) {
    timerAlarm(g_timer2, STEP_TIMER_HZ / g_step_hz_r, true, 0);
    clockPulse(CLOCK_R);
    g_step_r += g_step_sign_r;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR isrL(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  if (g_motor_move) {
    timerAlarm(g_timer3, STEP_TIMER_HZ / g_step_hz_l, true, 0);
    clockPulse(CLOCK_L);
    g_step_l += g_step_sign_l;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

static void stepperTimerStart(void)
{
  timerStart(g_timer2);
  timerStart(g_timer3);
}

static void stepperTimerStop(void)
{
  timerStop(g_timer2);
  timerStop(g_timer3);
}

static unsigned short stepHzClamp(double hz)
{
  hz = fabs(hz);
  if (hz < STEP_HZ_MIN) return STEP_HZ_MIN;
  if (hz > 65535.0) return 65535;
  return (unsigned short)hz;
}

void stepperDirSet(bool forward_l, bool forward_r)
{
  portENTER_CRITICAL(&g_timer_mux);
  digitalWrite(CW_L, forward_l ? CW_L_FORWARD : !CW_L_FORWARD);
  digitalWrite(CW_R, forward_r ? CW_R_FORWARD : !CW_R_FORWARD);
  g_step_sign_l = forward_l ? 1 : -1;
  g_step_sign_r = forward_r ? 1 : -1;
  portEXIT_CRITICAL(&g_timer_mux);
}

// 左右のパルス周波数[Hz]を設定し、パルス出力を始める
void stepperHzSet(double hz_l, double hz_r)
{
  g_step_hz_l = stepHzClamp(hz_l);
  g_step_hz_r = stepHzClamp(hz_r);
  g_motor_move = true;
}

void stepperStop(void) { g_motor_move = false; }

void stepperCounterClear(void)
{
  portENTER_CRITICAL(&g_timer_mux);
  g_step_l = 0;
  g_step_r = 0;
  portEXIT_CRITICAL(&g_timer_mux);
}

void stepperCounterGet(int * l, int * r)
{
  portENTER_CRITICAL(&g_timer_mux);
  *l = g_step_l;
  *r = g_step_r;
  portEXIT_CRITICAL(&g_timer_mux);
}


// GPIO、タイマー、シリアル通信を初期化
void deviceInit(void)
{
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  ledSet(0);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT);  // 内部プルアップなし

  ledcAttach(BUZZER, 440, 10);
  ledcWrite(BUZZER, 1024);

  pinMode(SLED_F, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_F, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);  // SLEEP
  pinMode(CW_R, OUTPUT);
  pinMode(CW_L, OUTPUT);
  pinMode(CLOCK_R, OUTPUT);
  pinMode(CLOCK_L, OUTPUT);
  digitalWrite(CLOCK_R, LOW);
  digitalWrite(CLOCK_L, LOW);
  stepperDirSet(true, true);

  g_timer0 = timerBegin(1000000);
  timerAttachInterrupt(g_timer0, &onTimer0);
  timerAlarm(g_timer0, 1000, true, 0);
  timerStart(g_timer0);

  g_timer2 = timerBegin(STEP_TIMER_HZ);
  timerAttachInterrupt(g_timer2, &isrR);
  timerAlarm(g_timer2, STEP_TIMER_HZ / STEP_HZ_MIN, true, 0);
  timerStart(g_timer2);

  g_timer3 = timerBegin(STEP_TIMER_HZ);
  timerAttachInterrupt(g_timer3, &isrL);
  timerAlarm(g_timer3, STEP_TIMER_HZ / STEP_HZ_MIN, true, 0);
  timerStart(g_timer3);

  // センサ計測は core1 固定タスクで実行 (旧 onTimer1 の置き換え)
  xTaskCreatePinnedToCore(
    sensorTask, "sensorTask", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIORITY, &g_sensor_task,
    SENSOR_TASK_CORE);

  Serial.begin(115200);
}

// dataの下位4ビットをLED0〜3に表示する(ビットが1なら点灯)
void ledSet(unsigned char data)
{
  digitalWrite(LED0, (data & 0x01) ? LED_ON : LED_OFF);
  digitalWrite(LED1, (data & 0x02) ? LED_ON : LED_OFF);
  digitalWrite(LED2, (data & 0x04) ? LED_ON : LED_OFF);
  digitalWrite(LED3, (data & 0x08) ? LED_ON : LED_OFF);
}

// ブザーを指定周波数で鳴らす
void buzzerEnable(short f) { ledcWriteTone(BUZZER, f); }

void buzzerDisable(void)
{
  ledcWrite(BUZZER, 1024);  //duty 100% Buzzer OFF
}

void motorEnable(void)
{
  digitalWrite(MOTOR_EN, HIGH);  //Power ON
}

void motorDisable(void)
{
  digitalWrite(MOTOR_EN, LOW);  //Power OFF (SLEEP)
}

// 押されたスイッチを取得し、離されるまで待つ。USE_SW2が1ならSW1=SW_RM(次)、SW2=SW_CM(決定)。
// 0ならSW1の短押しがSW_RM、SW_LONG_PRESS_MS以上の長押しがSW_CM。長押しになった時点で短く鳴らす
unsigned char switchGet(void)
{
  unsigned char ret = 0;
  if (digitalRead(SW1) == LOW) {
    const unsigned long t0 = millis();
    bool long_press = false;
    do {
      delay(20);
      if (!USE_SW2 && !long_press && (millis() - t0 >= SW_LONG_PRESS_MS)) {
        long_press = true;
        buzzerEnable(DEC_FREQ);
        delay(30);
        buzzerDisable();
      }
    } while (digitalRead(SW1) == LOW);
    ret |= long_press ? SW_CM : SW_RM;
  }
#if USE_SW2
  if (digitalRead(SW2) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW2) == LOW);
    ret |= SW_CM;
  }
#endif
  return ret;
}

// SLED点灯 -> 整定待ち -> analogRead。点灯窓がSLED_WINDOW_MAX_USを超えて伸びたら
// SENSOR_SAMPLE_BADを返し、呼び出し側で棄却する(STEP8_動作まとめ.md §7参照)
static unsigned short sledRead(uint8_t sled_pin, uint8_t adc_pin)
{
  digitalWrite(sled_pin, HIGH);
  uint32_t t0 = micros();
  delayMicroseconds(SLED_SETTLE_US);
  unsigned short tmp = analogRead(adc_pin);
  uint32_t on_us = micros() - t0;
  digitalWrite(sled_pin, LOW);
  return (on_us > SLED_WINDOW_MAX_US) ? SENSOR_SAMPLE_BAD : tmp;
}

unsigned short sensorGetR(void) { return sledRead(SLED_R, AD_R); }
unsigned short sensorGetL(void) { return sledRead(SLED_L, AD_L); }
unsigned short sensorGetF(void) { return sledRead(SLED_F, AD_F); }

// 電池電圧[mV]を返す。51kと10kの分圧を戻して計算する
short batteryVoltGet(void) { return (double)analogReadMilliVolts(AD_BATT) / 10.0 * (10.0 + 51.0); }
