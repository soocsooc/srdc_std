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

#ifndef STEPPER_H_
#define STEPPER_H_

// EIC4311(SLA7078互換)のパルス生成。v3(Pi:Co Classic3)と同じく、左右のタイマー割り込みで
// CLOCKピンにパルスを出し、パルス数を数える。カウンタは前進方向を正とする符号付き
void stepperInit(void);
void stepperDirSet(bool forward_l, bool forward_r);
void stepperHzSet(double hz_l, double hz_r);  // 左右のパルス周波数[Hz]を設定し、出力を始める
void stepperStop(void);
void stepperCounterClear(void);
void stepperCounterGet(int * l, int * r);

#endif  // STEPPER_H_
