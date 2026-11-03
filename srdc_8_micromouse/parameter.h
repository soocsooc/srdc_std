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
#ifndef PARAMETER_H_
#define PARAMETER_H_

// 初期パラメータ。標準機体では未校正のため、v3(Pi:Co Classic3)の値を仮に入れてある。
// タイヤ径・トレッド幅・センサ値は調整メニュー(モード7)とWeb調整で校正すること
#define TIRE_DIAMETER_INIT (48.0)
#define TREAD_WIDTH_INIT (65.0)

// モータ1回転あたりのパルス数。EIC4311(SLA7078互換)をM2=H・M1/M3=Lで使う前提の仮の値で、
// 実機で1回転させて確認すること
#define PULSE_PER_REV 400.0

#define REF_SEN_R_INIT 589
#define REF_SEN_L_INIT 628

#define TH_SEN_R_INIT 300
#define TH_SEN_L_INIT 215
#define TH_SEN_F_INIT 103
// 調整モード10で区画中央に置き、1区画先の前壁あり/なしで測った前センサ値(ベース値、設計メモ §7)
#define SEN_F_WALL_INIT 150
#define SEN_F_NOWALL_INIT 56

// ---- センサ動的到達判定版(モード2・3、run_dynamic.cpp/search_dynamic.cpp)のパラメータ ----
// 詳細は設計メモ §4参照。

// 走行中の前壁到達判定閾値(前センサの値、設計メモ §4)。モード2は区画境界で止まる前提。
// モード3も区画境界で判定するが、速度が違うため別値にしてある(設計メモ §4)。
#define TH_SEN_F_APPROACH_INIT 145
#define TH_SEN_F_APPROACH_SLALOM_INIT 70

// モード3の側方壁判定閾値。TH_SEN_R/L_INITは真の区画中心での停止前提のため別枠(設計メモ §5)。
#define TH_SEN_R_SLALOM_INIT TH_SEN_R_INIT
#define TH_SEN_L_SLALOM_INIT TH_SEN_L_INIT

// センサのベース値(REF・しきい値)に掛ける倍率[%]。会場のセンサ感度に合わせて一括で変える(設計メモ §12)
#define SEN_RATIO_INIT 100
#define SEN_RATIO_MIN 10
#define SEN_RATIO_MAX 500

// 迷路の大きさ[区画]。Web UIで2〜MAZESIZE_MAXの範囲で変更できる(設計メモ §9)
#define MAZESIZE_X_INIT 4
#define MAZESIZE_Y_INIT 4
// ゴール区画は(GOAL_X,GOAL_Y)-(GOAL_X2,GOAL_Y2)の長方形(対角2座標、設計メモ §9)
#define GOAL_X_INIT 3
#define GOAL_Y_INIT 3
#define GOAL_X2_INIT 3
#define GOAL_Y2_INIT 3

// 壁センサLEDパルスの整定待ち[us]。傾きの緩いRC裾で読み、読み取りタイミングの
// 揺れの影響を抑える(STEP8_動作まとめ.md §7参照)
#define SLED_SETTLE_US 32
// パルス跨ぎIIRの係数 a : value = (value*a + in)/(a+1)  (a=3 -> 時定数≒3ms @1kHz/ch)
#define SEN_IIR_A 3

// SLED点灯からAD確定までの許容時間[us]。これを超えて窓が伸びたサンプルは棄却し、
// 前回値を維持する。調整方法はSTEP8_動作まとめ.md §7参照
#define SLED_WINDOW_MAX_US 200
// 棄却サンプルを表す番兵値 (analogReadは0-4095なので衝突しない)
#define SENSOR_SAMPLE_BAD 0xFFFF

#define CON_WALL_KP_INIT 0.3
#define SEARCH_ACCEL_INIT 1.5
#define TURN_ACCEL_INIT 0.4
#define SEARCH_SPEED_INIT 600
#define MAX_SPEED_INIT 800
#define MIN_SPEED 30
// slalom()の旋回中の並進速度(外輪の基準速度)。旋回の軌跡は速度によらない(設計メモ §3)
#define SLALOM_SPEED_INIT 400

// slalom()の旋回形状(設計メモ §3)。外輪=速度×(1+OUTER_GAIN×f)、内輪=速度×(1−(1−INNER_RATIO)×f)。
// fは外輪の走行距離RAMP_LEN[mm]で0→1に上げ、出口で対称に1→0へ下げる。
#define SLALOM_INNER_RATIO_INIT 0.1
#define SLALOM_OUTER_GAIN_INIT 0.0
#define SLALOM_RAMP_LEN_INIT 30.0
#define SLALOM_INNER_RATIO_MAX 0.9
#define SLALOM_OUTER_GAIN_MAX 1.0
#define SLALOM_RAMP_LEN_MAX 150.0
// fの下限。0だと旋回が始まらないため
#define SLALOM_F_MIN 0.05
// 内輪をMIN_SPEEDまで持ち上げるときに左右へ掛ける倍率の上限(inner_ratioが0付近のとき用、設計メモ §3)
#define SLALOM_MIN_SPEED_SCALE_MAX 2.0


#define INC_FREQ 3000
#define DEC_FREQ 2000

// LiPo 3セル。残量LEDはないため、低電圧の警告にだけ使う
#define BATT_MIN 10000
// 電池電圧がこのミリ秒だけ連続で BATT_MIN 未満だったとき低電圧ブザーを鳴らす。
// WiFi TX の瞬間的なレール低下(数ms)で誤発報しないためのデバウンス。
#define BATT_LOW_ALARM_MS 200

#define HALF_SECTION 90
#define SECTION 180

// 行き止まりUターン・走行開始時に壁へ当てるバックの距離[mm]と速度[mm/s]。バックは壁へ確実に
// 当てるため前進より長くする(設計メモ §6)
#define UTURN_BACK_LEN_INIT 40
// 壁に当てた後、区画中心へ戻る前進距離[mm](uTurn()・alignStartPosition()共通)
#define WALL_RETURN_LEN_INIT 25
#define UTURN_SPEED_INIT 100

// paramRead()の妥当性チェックの上限。超過・0以下は*_INITへ戻す(設計メモ §12参照)
#define SPEED_SANITY_MAX 3000   // search_speed/max_speedの上限[mm/s]
#define ACCEL_SANITY_MAX 100.0  // search_accel/turn_accelの上限

// rotate()/slalom()のタイムアウト[ms]。旋回が終わらないときに強制停止する(設計メモ §8参照)
#define TURN_TIMEOUT_MS 3000

#endif  // PARAMETER_H_