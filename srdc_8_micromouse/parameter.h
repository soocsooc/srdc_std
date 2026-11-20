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

#define TIRE_DIAMETER_INIT (51.66)  // タイヤ径[mm]、直進距離に影響
#define TREAD_WIDTH_INIT (100.95)   // トレッド幅[mm]、旋回角度に影響

// モータ1回転あたりのパルス数
// 1.8°モータ、EIC4311(SLA7078互換)はM2=H・M1/M3=Lの2分割なので200×2
#define PULSE_PER_REV 400.0

// ---- 壁センサ ----
// 下の基準値・閾値には判定時にSEN_RATIOの倍率を掛ける

// 区画中心で測った前センサ値(1区画先の前壁あり/なし)。調整モード10で測定(設計メモ §7)
#define SEN_F_WALL_INIT 125
#define SEN_F_NOWALL_INIT 35

// 機体が左右の中央にあるときの横センサ値。壁制御の目標値。区画中心では光が柱に当たるので、
// 前後にずらして光が壁に当たる位置で測る(設計メモ §14)
#define REF_SEN_R_INIT 201
#define REF_SEN_L_INIT 240

// 壁があると判断する閾値
#define TH_SEN_R_INIT 75
#define TH_SEN_L_INIT 101
// 前センサは区画境界で判定する。境界で前壁(174mm先)が約335、2区画先の壁(354mm先)だけなら約47なので、
// その間で見落としにくい低めの値にした(調整モード8のログ、設計メモ §7)
#define TH_SEN_F_INIT 120

// 横センサが柱を読んだときの値の落ち込みの判定(設計メモ §14)。落ち込み中は負の誤差を制御に使わない
// 400mm/sで柱の落ち込みは約16mm(40ms)かけて30〜40下がる。比べる間隔が短いとノイズと区別できない
#define SEN_DROP_LOOKBACK 20  // 何ms前の値と比べるか(400mm/sで8mm手前)
#define SEN_DROP_PCT 3  // 基準値の何%より大きく下がったら落ち込みとみなすか
#define SEN_DIP_MAX_LEN 30  // 落ち込みの最長距離[mm]。柱で値が下がる区間は約16mm

// 走行中に前壁が近づいたと判断する前センサの閾値(設計メモ §4)
// モード2用とモード3用。速度が違うため別の値にしてある
// 区画境界で前壁(174mm先)を見たときの値。調整モード8のログで測った
#define TH_SEN_F_APPROACH_INIT 330
#define TH_SEN_F_APPROACH_SLALOM_INIT TH_SEN_F_APPROACH_INIT

// モード1の直進で前壁の見落としに気付いたとき、区画中心を越えて減速してよい距離の上限[mm](設計メモ §15)
#define FRONT_GUARD_OVERRUN_MAX 20

// モード3の横壁の閾値(th_sen_r/l_slalom)の初期値はTH_SEN_R/L_INITと同じ(設計メモ §5)

// 基準値・閾値に掛ける倍率[%]。会場でセンサの感度が変わったときに一括で合わせる(設計メモ §12)
#define SEN_RATIO_INIT 100
#define SEN_RATIO_MIN 10
#define SEN_RATIO_MAX 500

// LED点灯からAD変換までの待ち時間[us]。受光の出力は点灯直後の減衰波形なので、傾きの緩い所で読み、
// 読むタイミングの揺れによる値のばらつきを抑える
#define SLED_SETTLE_US 32

// 1ならLEDを点ける直前に消灯時の値(主に受光回路のバイアス、210前後)を読み、点灯時の値から引く(設計メモ §15)。
// 値の大きさが変わるので、切り替えたら横センサの基準値と全センサのしきい値を測り直すこと
#define SEN_AMBIENT_CANCEL 1

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

#define CON_WALL_KP_INIT 0.5   // 壁制御の比例ゲイン
#define SEARCH_ACCEL_INIT 1.0  // 直進の加速度[m/s^2]
#define TURN_ACCEL_INIT 1.0    // 超信地旋回の加速度[m/s^2]
#define SEARCH_SPEED_INIT 400  // 探索の速度[mm/s]
#define MAX_SPEED_INIT 800     // 最短走行の最高速度[mm/s]
#define MIN_SPEED 30           // 最低速度[mm/s]
#define STOP_WAIT_MS 300       // stop()で機体が止まるまで待つ時間[ms]

// スラロームの旋回中の速度[mm/s](外輪の基準)。旋回の軌跡は速度によらない(設計メモ §3)
#define SLALOM_SPEED_INIT 400

// スラロームの旋回の形(設計メモ §3)
// 外輪 = 速度×(1 + OUTER_GAIN×f)、内輪 = 速度×(1 − (1 − INNER_RATIO)×f)
// fは旋回の強さ。外輪がRAMP_LEN[mm]走る間にfを0→1へ上げ、旋回の終わりに同じ距離で1→0へ下げる
// 内輪の速度の比。0.1で一定区間の旋回半径は約57mm、張り出しは約72mm(SLALOM_TREAD_WIDTH_INITの約92.5mm、設計メモ §3)
#define SLALOM_INNER_RATIO_INIT 0.1
#define SLALOM_OUTER_GAIN_INIT 0.0
#define SLALOM_RAMP_LEN_INIT 30.0
// SLALOM_TREAD_WIDTH_INITを決めるためTREAD_WIDTH_INITに掛ける比。実測で合わせた92.5mm ÷ 100.95mm
#define SLALOM_TREAD_RATIO 0.9163
// スラロームの旋回角の計算に使うトレッド幅[mm]。0ならTREAD_WIDTHと同じ値を使う。曲がりながら走るときの
// 実際のトレッド幅は超信地旋回のときと違うことがあるので別に持つ。旋回の終わりで曲がりすぎるなら小さくする(設計メモ §3)
#define SLALOM_TREAD_WIDTH_INIT (TREAD_WIDTH_INIT * SLALOM_TREAD_RATIO)
#define SLALOM_TREAD_WIDTH_MAX 300.0
#define SLALOM_INNER_RATIO_MAX 0.9
#define SLALOM_OUTER_GAIN_MAX 1.0
#define SLALOM_RAMP_LEN_MAX 150.0
// fの最小値。0だと旋回が始まらない
#define SLALOM_F_MIN 0.05
// 内輪がMIN_SPEEDを下回るとき、左右の速度に同じ倍率を掛けて上げる。その倍率の上限
#define SLALOM_MIN_SPEED_SCALE_MAX 2.0
// slalomCell(dir, true)で表示する走行の段階(調整モード3・9)。直進は外側2個、スラローム中は内側2個を点ける
#define LED_PHASE_STRAIGHT 0b1001
#define LED_PHASE_SLALOM 0b0110

// 後ろの壁に当てるときのバック距離[mm]。確実に当てるため前進より長くする
// (行き止まりのUターンと走行開始時、設計メモ §6)
#define UTURN_BACK_LEN_INIT 40
// 壁に当てた後、区画中心へ戻る前進距離[mm]
#define WALL_RETURN_LEN_INIT 32
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
