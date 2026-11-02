#ifndef PINS_CONFIG_H_
#define PINS_CONFIG_H_

// 受光素子(ADC1)。前・右・左の対応は仮置きのため、実機で確認すること
#define AD_F 36
#define AD_R 39
#define AD_L 34
#define AD_BATT 32

// センサ用LEDピン(HIGHで点灯)。前は1個
#define SLED_F 14
#define SLED_R 4
#define SLED_L 27

// EIC4311(SLA7078互換)の駆動ピン
#define CLOCK_R 19
#define CLOCK_L 18
#define CW_R 25
#define CW_L 26
// 前進時のCW/CCWピンの出力。モータの向きが逆なら、ここを反転する
#define CW_R_FORWARD LOW
#define CW_L_FORWARD LOW

#define TIRE_DIAMETER (48.00)
#define TREAD_WIDTH (65.00)
#define MIN_SPEED 30

// モータ1回転あたりのパルス数(M2=H・M1/M3=Lの前提の仮の値。実機で1回転させて確認する)
#define PULSE_PER_REV 400.0
#define PULSE (TIRE_DIAMETER * PI / PULSE_PER_REV)

//環境に合わせて変更する(標準機体では未校正。STEP7のSW1でセンサ値を表示して決める)
//ここから
#define REF_SEN_R 589
#define REF_SEN_L 628

#define TH_SEN_R 300
#define TH_SEN_L 215
#define TH_SEN_F 103

#define CONTH_SEN_R TH_SEN_R
#define CONTH_SEN_L TH_SEN_L

#define CON_WALL_KP (0.5)
//ここまで

#endif
