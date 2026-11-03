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

#include <Arduino.h>

// SRDC2026標準機体(ESP32-DevKitC-32E)のピン定義。回路図V5(11/3版)の配線に合わせてある

// switchGet()の戻り値。SW_RM=次、SW_CM=決定。−1の操作はない
#define SW_CM 2
#define SW_RM 4

// LEDはカソードがGND側のため、HIGHで点灯する。LED0〜3は回路図のL_1〜L_4
#define LED0 5
#define LED1 13
#define LED2 16
#define LED3 15
#define LED_ON HIGH
#define LED_OFF LOW

// SW1は内部プルアップを使う。SW2(GPIO35)は入力専用ピンで内部プルアップがなく、基板にも
// プルアップがないため、押していないときの値が定まらない。基板を直して3.3Vへの10kの
// プルアップを付けるまでは、SW2を読まず、SW1の短押しを「次」、長押しを「決定」にする。
// 基板を直したらUSE_SW2を1にすると、SW1が「次」、SW2が「決定」になる
#define SW1 33
#define SW2 35
#define USE_SW2 0
#define SW_LONG_PRESS_MS 1000  // SW1をこの時間以上押したら長押し(決定)

// センサLED(HIGHで点灯)。前は1個で、前方の受光素子1個と組になる
#define SLED_F 14
#define SLED_R 4
#define SLED_L 27

// 受光素子(すべてADC1)。前・右・左の対応は仮置きのため、実機で確認すること
#define AD_F 36  // S1
#define AD_R 39  // S3
#define AD_L 34  // S4
#define AD_BATT 32

#define BUZZER 23

// EIC4311(SLA7078互換)。MOTOR_ENはHIGHで有効(LOWでREF/SLEEPがSLEEPになる)
#define MOTOR_EN 17
#define CLOCK_R 19
#define CLOCK_L 18
#define CW_R 22
#define CW_L 21
// 前進時のCW/CCWピンの出力。モータの向きが逆なら、ここを反転する
#define CW_R_FORWARD LOW
#define CW_L_FORWARD LOW

// device.cpp で定義される関数群(旧 .ino 連結時の自動プロトタイプ生成の代替)
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

// ステッピングモータのパルス生成(device.cpp)。カウンタは前進方向を正とする符号付き
void stepperDirSet(bool forward_l, bool forward_r);
void stepperHzSet(double hz_l, double hz_r);
void stepperStop(void);
void stepperCounterClear(void);
void stepperCounterGet(int * l, int * r);

#endif  // DEVICE_H_
