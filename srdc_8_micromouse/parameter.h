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

// 各パラメータの初期値
// タイヤ径・トレッド幅・センサ値は調整メニュー(モード7)とWeb調整で校正

#define TIRE_DIAMETER_INIT (48.0)  // タイヤ径[mm]、直進距離に影響
#define TREAD_WIDTH_INIT (65.0)    // トレッド幅[mm]、旋回角度に影響

// モータ1回転あたりのパルス数
// EIC4311(SLA7078互換)をM2=H・M1/M3=Lで使う前提の仮の値のため、実機で1回転させて確認
#define PULSE_PER_REV 400.0

// ---- 壁センサ ----
// 下の基準値・閾値には判定時にSEN_RATIOの倍率を掛ける

// 機体が区画中心にあるときの横センサ値。壁制御の目標値
#define REF_SEN_R_INIT 589
#define REF_SEN_L_INIT 628
// 壁があると判断する閾値
#define TH_SEN_R_INIT 300
#define TH_SEN_L_INIT 215
#define TH_SEN_F_INIT 103
// 区画中心で測った前センサ値(1区画先の前壁あり/なし)。調整モード10で測定(設計メモ §7)
#define SEN_F_WALL_INIT 150
#define SEN_F_NOWALL_INIT 56

// 走行中に前壁が近づいたと判断する前センサの閾値(設計メモ §4)
// モード2用とモード3用。速度が違うため別の値にしてある
#define TH_SEN_F_APPROACH_INIT 145
#define TH_SEN_F_APPROACH_SLALOM_INIT 70

// モード3の横壁の閾値(th_sen_r/l_slalom)の初期値はTH_SEN_R/L_INITと同じ(設計メモ §5)

// 基準値・閾値に掛ける倍率[%]。会場でセンサの感度が変わったときに一括で合わせる(設計メモ §12)
#define SEN_RATIO_INIT 100
#define SEN_RATIO_MIN 10
#define SEN_RATIO_MAX 500

// LED点灯からAD変換までの待ち時間[us]。受光の出力は点灯直後の減衰波形なので、傾きの緩い所で読み、
// 読むタイミングの揺れによる値のばらつきを抑える
#define SLED_SETTLE_US 32
// センサ値の平滑化の係数a。value = (value*a + 新しい値)/(a+1)
#define SEN_IIR_A 3
// LED点灯からAD変換が終わるまでの許容時間[us]。超えた読み取り値は捨てて前回値を使う
// (STEP8_GUIDE.md §7)
#define SLED_WINDOW_MAX_US 200
// 捨てる読み取り値を表す値。analogRead()は0〜4095なので重ならない
#define SENSOR_SAMPLE_BAD 0xFFFF

// ---- 迷路 ----

// 迷路の大きさ[区画]。Web UIで2〜MAZESIZE_MAXの範囲で変更できる(設計メモ §9)
#define MAZESIZE_X_INIT 4
#define MAZESIZE_Y_INIT 4
// ゴール区画。(GOAL_X,GOAL_Y)と(GOAL_X2,GOAL_Y2)を対角とする長方形(設計メモ §9)
#define GOAL_X_INIT 3
#define GOAL_Y_INIT 3
#define GOAL_X2_INIT 3
#define GOAL_Y2_INIT 3

#define HALF_SECTION 90  // 半区画[mm]
#define SECTION 180      // 1区画[mm]

// ---- 走行 ----

#define CON_WALL_KP_INIT 0.3   // 壁制御の比例ゲイン
#define SEARCH_ACCEL_INIT 1.5  // 直進の加速度[m/s^2]
#define TURN_ACCEL_INIT 0.4    // 超信地旋回の加速度[m/s^2]
#define SEARCH_SPEED_INIT 600  // 探索の速度[mm/s]
#define MAX_SPEED_INIT 800     // 最短走行の最高速度[mm/s]
#define MIN_SPEED 30           // 最低速度[mm/s]
#define STOP_WAIT_MS 300       // stop()で機体が止まるまで待つ時間[ms]

// スラロームの旋回中の速度[mm/s](外輪の基準)。旋回の軌跡は速度によらない(設計メモ §3)
#define SLALOM_SPEED_INIT 400

// スラロームの旋回の形(設計メモ §3)
// 外輪 = 速度×(1 + OUTER_GAIN×f)、内輪 = 速度×(1 − (1 − INNER_RATIO)×f)
// fは旋回の強さ。外輪がRAMP_LEN[mm]走る間にfを0→1へ上げ、旋回の終わりに同じ距離で1→0へ下げる
#define SLALOM_INNER_RATIO_INIT 0.1
#define SLALOM_OUTER_GAIN_INIT 0.0
#define SLALOM_RAMP_LEN_INIT 30.0
#define SLALOM_INNER_RATIO_MAX 0.9
#define SLALOM_OUTER_GAIN_MAX 1.0
#define SLALOM_RAMP_LEN_MAX 150.0
// fの最小値。0だと旋回が始まらない
#define SLALOM_F_MIN 0.05
// 内輪がMIN_SPEEDを下回るとき、左右の速度に同じ倍率を掛けて上げる。その倍率の上限
#define SLALOM_MIN_SPEED_SCALE_MAX 2.0

// 後ろの壁に当てるときのバック距離[mm]。確実に当てるため前進より長くする
// (行き止まりのUターンと走行開始時、設計メモ §6)
#define UTURN_BACK_LEN_INIT 40
// 壁に当てた後、区画中心へ戻る前進距離[mm]
#define WALL_RETURN_LEN_INIT 25
// バックと前進の速度[mm/s]
#define UTURN_SPEED_INIT 100

// 保存値の範囲チェックの上限。範囲外なら初期値に戻す(設計メモ §12)
#define SPEED_SANITY_MAX 3000   // 速度の上限[mm/s](下限はMIN_SPEED)
#define ACCEL_SANITY_MAX 100.0  // 加速度の上限[m/s^2]

// 旋回のタイムアウト[ms]。旋回が終わらないときに止める(設計メモ §8)
#define TURN_TIMEOUT_MS 3000

// ---- その他 ----

// ブザーの音程[Hz]
#define INC_FREQ 3000
#define DEC_FREQ 2000

// 電池電圧の下限[mV](LiPo 3セル)。下回ると低電圧のブザーを鳴らす
#define BATT_MIN 10000
// BATT_MINをこの時間[ms]続けて下回ったときだけ鳴らす。WiFi送信で電圧が一瞬下がっても鳴らないようにする
#define BATT_LOW_ALARM_MS 200

#endif  // PARAMETER_H_
