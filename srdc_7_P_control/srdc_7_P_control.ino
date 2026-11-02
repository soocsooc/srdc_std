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

#include "pins_config.h"
#include "stepper.h"
#include "run.h"
#include "sensor.h"

// LEDはLOWで点灯する
#define LED0 5
#define LED1 13
#define LED2 15
#define LED3 16

// SW2(GPIO35)は内部プルアップがないため、10kの外付けプルアップが必要
#define SW1 33
#define SW2 35

// HIGHでモータ有効(LOWでドライバがSLEEP)
#define MOTOR_EN 17

// ADピン・SLEDピン・駆動ピン・タイヤ径・PULSE・MIN_SPEED および
// 環境調整パラメータ(REF_SEN/TH_SEN/CON_WALL_KP 等)は pins_config.h に集約

hw_timer_t * g_timer0 = NULL;
hw_timer_t * g_timer1 = NULL;
portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

//割り込み
//目標値の更新周期1kHz
void IRAM_ATTR onTimer0(void) // モータ制御
{
  portENTER_CRITICAL_ISR(&g_timer_mux);  //割り込み禁止
  controlInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);  //割り込み許可
}

void IRAM_ATTR onTimer1(void) // センサ制御
{
  portENTER_CRITICAL_ISR(&g_timer_mux);  //割り込み禁止
  sensorInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);  //割り込み許可
}

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

  //motor disable
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);

  pinMode(SLED_F, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_F, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  Serial.begin(115200);

  g_timer0 = timerBegin(1000000);  //1MHz(1us)
  timerAttachInterrupt(g_timer0, &onTimer0);
  timerAlarm(g_timer0, 1000, true, 0);  //1000 * 1us =1000us(1kHz)
  timerStart(g_timer0);

  g_timer1 = timerBegin(1000000);
  timerAttachInterrupt(g_timer1, &onTimer1);
  timerAlarm(g_timer1, 250, true, 0);
  timerStart(g_timer1);

  stepperInit();
}

void loop()
{
  // put your main code here, to run repeatedly:
  while (digitalRead(SW1) & digitalRead(SW2)) { // スイッチが押されるまで待機
    continue;
  }
  if (digitalRead(SW1) == 0) { // SW1でセンサ値の表示(SW2で走行)
    while (1) {
      Serial.printf("r_sen  is %d\n\r", g_sensor.sen_r.value);
      Serial.printf("f_sen  is %d\n\r", g_sensor.sen_f.value);
      Serial.printf("l_sen  is %d\n\r", g_sensor.sen_l.value);
      Serial.printf("VDD    is %d\n\r", g_sensor.battery_value);
      delay(100);
    }
  }
  digitalWrite(MOTOR_EN, HIGH);  // 励磁ON
  delay(1000);
  g_run.accelerate(90, 350);		// 0.5区画加速
  g_run.oneStep(180 * 3, 350);	// 3区画前進
  g_run.decelerate(90, 350);		// 0.5区画減速
  delay(1000);
  digitalWrite(MOTOR_EN, LOW);  // 励磁OFF
}
