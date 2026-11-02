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

// SRDC2026標準機体(ESP32-DevKitC-32E)のピン定義。基板(10/29版)の配線に合わせてある

// switchGet()の戻り値。SW1=次(SW_RM)、SW2=決定(SW_CM)。−1の操作はない
#define SW_CM 2
#define SW_RM 4

// LEDはアノードが3.3V側のため、LOWで点灯する
#define LED0 5
#define LED1 13
#define LED2 15
#define LED3 16
#define LED_ON LOW
#define LED_OFF HIGH

// SW1は内部プルアップを使う。SW2(GPIO35)は入力専用ピンで内部プルアップがないため、
// 10kの外付けプルアップが必要
#define SW1 33
#define SW2 35

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
#define CW_R 25
#define CW_L 26
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
