#ifndef PINS_CONFIG_H_
#define PINS_CONFIG_H_

// EIC4311(SLA7078互換)の駆動ピン
#define CLOCK_R 19
#define CLOCK_L 18
#define CW_R 22
#define CW_L 21
// 前進時のCW/CCWピンの出力。モータの向きが逆なら、ここを反転する
#define CW_R_FORWARD LOW
#define CW_L_FORWARD LOW

#define TIRE_DIAMETER (48.00)
#define TREAD_WIDTH (65.00)
#define MIN_SPEED 30

// モータ1回転あたりのパルス数(M2=H・M1/M3=Lの前提の仮の値。実機で1回転させて確認する)
#define PULSE_PER_REV 400.0
#define PULSE (TIRE_DIAMETER * PI / PULSE_PER_REV)

#endif
