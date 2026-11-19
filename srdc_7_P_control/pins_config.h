#ifndef PINS_CONFIG_H_
#define PINS_CONFIG_H_

// 受光素子(ADC1)
#define AD_F 39
#define AD_R 34
#define AD_L 36
#define AD_BATT 35

// センサ用LEDピン(HIGHで点灯)。前は1個
#define SLED_F 14
#define SLED_R 4
#define SLED_L 27

// EIC4311(SLA7078互換)の駆動ピン
#define CLOCK_R 19
#define CLOCK_L 18
#define CW_R 22
#define CW_L 21
// 前進時のCW/CCWピンの出力。モータの向きが逆なら、ここを反転する
#define CW_R_FORWARD HIGH
#define CW_L_FORWARD LOW

#define TIRE_DIAMETER (51.66)
#define TREAD_WIDTH (101.00)
#define MIN_SPEED 30

// モータ1回転あたりのパルス数(1.8°モータ、EIC4311はM2=H・M1/M3=Lの2分割で200×2)
#define PULSE_PER_REV 400.0
#define PULSE (TIRE_DIAMETER * PI / PULSE_PER_REV)

//環境に合わせて変更する(標準機体では未校正。STEP7のSW1の短押しでセンサ値を表示して決める)
//ここから
#define REF_SEN_R 757
#define REF_SEN_L 592

#define TH_SEN_R 385
#define TH_SEN_L 350
#define TH_SEN_F 447

#define CON_WALL_KP (0.5)
//ここまで

#endif
