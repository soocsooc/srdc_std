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
#ifndef DEVICE_H_
#define DEVICE_H_

#include <Arduino.h>  // IWYU pragma: keep (マクロの中身でHIGH/LOWを使う)

// SRDC2026標準機体(ESP32-DevKitC-32E)のピン定義。回路図(11/11版)の配線に合わせてある

// switchGet()の戻り値。SW_RM=次、SW_CM=決定(値を減らす操作はない)
#define SW_CM 2
#define SW_RM 4

// LEDはカソードがGND側のため、HIGHで点灯する。LED0〜3は回路図のL_1〜L_4
#define LED0 5
#define LED1 13
#define LED2 16
#define LED3 15
#define LED_ON HIGH
#define LED_OFF LOW

// SW1が「次」、SW2が「決定」。どちらも内部プルアップを使う
#define SW1 33
#define SW2 32

// センサLED(HIGHで点灯)。前は1個で、前方の受光素子1個と組になる
#define SLED_F 14
#define SLED_R 4
#define SLED_L 27

// 受光素子(すべてADC1)
#define AD_F 39  // S3
#define AD_R 34  // S4
#define AD_L 36  // S1
#define AD_BATT 35

#define BUZZER 23

// EIC4311(SLA7078互換)。MOTOR_ENはHIGHで有効(LOWでREF/SLEEPがSLEEPになる)
#define MOTOR_EN 17
#define CLOCK_R 19
#define CLOCK_L 18
#define CW_R 22
#define CW_L 21
// 前進時のCW/CCWピンの出力。モータの向きが逆なら、ここを反転する
#define CW_R_FORWARD HIGH
#define CW_L_FORWARD LOW

// device.cppの関数
// onTimer0/isrR/isrL は device.cpp 内でのみ使用するためここには宣言しない
void controlInterruptStart(void);
void controlInterruptStop(void);
void sensorInterruptStart(void);
void sensorInterruptStop(void);
void deviceInit(void);
void ledSet(unsigned char data);
void buzzerEnable(short f);
void buzzerDisable(void);
void motorEnable(void);
void motorDisable(void);
unsigned char switchGet(void);
unsigned short sensorGetR(void);
unsigned short sensorGetL(void);
unsigned short sensorGetF(void);
short batteryVoltGet(void);
extern volatile int g_core_timer0;  // onTimer0が動いたコア(未実行なら-1)
extern volatile int g_core_sensor;  // sensorTaskが動いているコア(未起動なら-1)

// ステッピングモータのパルス生成(device.cpp)。カウンタは前進方向を正とする符号付き
void stepperDirSet(bool forward_l, bool forward_r);
void stepperHzSet(double hz_l, double hz_r);
void stepperStop(void);
void stepperCounterClear(void);
void stepperCounterGet(int * l, int * r);

#endif  // DEVICE_H_
