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

#include "pico_common.h"

RUN g_run;

RUN::RUN()
{
  speed = 0.0;
  accel = 0.0;
  turn_f = 0.0;
  turn_enable = false;
}

// 1kHzの制御割り込み(onTimer0)から呼ばれ、RUN::interrupt()で速度と壁制御を更新する
void controlInterrupt(void) { g_run.interrupt(); }

void RUN::interrupt(void)
{

  // 割り込み周期ごとに速度を更新し、上下限で制限

  speed += accel;

  if (speed > upper_speed_limit) {
    speed = upper_speed_limit;
  }
  if (speed < lower_speed_limit) {
    speed = lower_speed_limit;
  }

  // 壁制御が有効なときは左右の壁誤差から左右速度を補正
  if (con_wall.enable == true) {
    if ((g_sensor.sen_r.is_control == true) && (g_sensor.sen_l.is_control == true)) {
      con_wall.error = g_sensor.sen_r.error - g_sensor.sen_l.error;
    } else {
      con_wall.error = 2.0 * (g_sensor.sen_r.error - g_sensor.sen_l.error);
    }
    con_wall.control = 0.001 * speed * con_wall.kp * con_wall.error;
    speed_target_r = speed + con_wall.control;
    speed_target_l = speed - con_wall.control;
  } else if (turn_enable == true) {
    // スラローム中: 外輪を増速し、内輪を減速する(turn_fは旋回ループが外輪の走行距離から更新、設計メモ §3)
    double outer = speed * (1.0 + slalom_outer_gain * turn_f);
    double inner = speed * (1.0 - (1.0 - slalom_inner_ratio) * turn_f);
    // 内輪がMIN_SPEEDで頭打ちになると軌跡がずれるため、左右の速度比を保って持ち上げる(設計メモ §3)
    if (inner < MIN_SPEED) {
      const double k = min(SLALOM_MIN_SPEED_SCALE_MAX, MIN_SPEED / max(inner, 1.0));
      outer *= k;
      inner *= k;
    }
    speed_target_r = (turn_sign > 0) ? outer : inner;
    speed_target_l = (turn_sign > 0) ? inner : outer;
  } else {
    // 壁制御を使わない回転中は左右を同じ速度にする
    speed_target_r = speed;
    speed_target_l = speed;
  }

  // 補正後の速度が最低速度を下回らないようにする
  if (speed_target_r < MIN_SPEED) speed_target_r = MIN_SPEED;
  if (speed_target_l < MIN_SPEED) speed_target_l = MIN_SPEED;
}

void RUN::dirSet(t_CW_CCW dir_left, t_CW_CCW dir_right)
{
  stepperDirSet(dir_left == MOT_FORWARD, dir_right == MOT_FORWARD);
}

// 走行パルスのカウンタをクリア
void RUN::counterClear(void) { stepperCounterClear(); }

// 左右の速度[mm/s]をパルス周波数に変換して出力する
void RUN::speedSet(double l_speed, double r_speed) { stepperHzSet(l_speed / pulse, r_speed / pulse); }

// 速度をl_speedに固定し、パルスカウンタを0にして等速で走り続ける
void RUN::stay(float l_speed)
{
  controlInterruptStop();
  upper_speed_limit = lower_speed_limit = speed = (double)l_speed;
  accel = 0.0;
  counterClear();
  speedSet(l_speed, l_speed);
  controlInterruptStart();
}

// 左右の走行パルスの合計と走行距離を取得
void RUN::stepGet(void)
{
  int l, r;
  stepperCounterGet(&l, &r);
  step_lr = abs(l) + abs(r);
  step_lr_len = (int)((float)step_lr / 2.0 * pulse);
}

// 左右の走行パルスを符号付きで個別に取得(スラロームの並進量・回転量の分離用)
void RUN::stepGetLR(void) { stepperCounterGet(&step_l, &step_r); }

// モーターを停止し、機械が止まるまで待機
void RUN::stop(void)
{
  stepperStop();
  delay(STOP_WAIT_MS);
}

// init_speedからmax_spまで加速して直進し、残りの距離が制動距離を切ったらfinish_speedまで減速する。
// finish_speedがMIN_SPEEDなら止まり、それ以外は等速のまま終える
void RUN::straight(int len, int init_speed, int max_sp, int finish_speed)
{
  int obj_step;

  // 走行パラメータを設定してから割り込み制御を開始
  controlInterruptStop();
  upper_speed_limit = (double)max_sp;
  accel = (double)search_accel;

  if (init_speed < MIN_SPEED) {
    speed = (double)MIN_SPEED;
  } else {
    speed = (double)init_speed;
  }
  if (finish_speed < MIN_SPEED) {
    finish_speed = MIN_SPEED;
  }
  lower_speed_limit = (double)finish_speed;

  con_wall.enable = true;
  speedSet(speed, speed);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / pulse);
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (
      // 残り距離で減速開始位置を判定
      (int)(len - step_lr_len) <
      (int)(((speed * speed) - (lower_speed_limit * lower_speed_limit)) / (2.0 * 1000.0 * accel))) {
      break;
    }
  }

  accel = -1.0 * search_accel;

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (step_lr > obj_step) {
      break;
    }
  }

  // 次の動作へつなげる速度があれば停止せず、その速度を維持
  if (finish_speed == MIN_SPEED) {
    stop();
  } else {
    stay(finish_speed);
  }
}

// 最小速度から指定速度まで加速して直進
void RUN::accelerate(int len, int finish_speed)
{
  int obj_step;

  controlInterruptStop();
  accel = search_accel;
  speed = lower_speed_limit = (double)MIN_SPEED;
  upper_speed_limit = (double)finish_speed;
  con_wall.enable = true;
  dirSet(MOT_FORWARD, MOT_FORWARD);
  speedSet(speed, speed);
  counterClear();
  obj_step = (int)((float)len * 2.0 / pulse);
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (step_lr > obj_step) {
      break;
    }
  }

  stay(finish_speed);
}

// 指定速度で一定距離を直進
void RUN::oneStep(int len, int init_speed)
{
  int obj_step;

  controlInterruptStop();
  accel = 0.0;
  speed = lower_speed_limit = upper_speed_limit = (double)init_speed;
  con_wall.enable = true;
  dirSet(MOT_FORWARD, MOT_FORWARD);
  speedSet(speed, speed);
  obj_step = (int)((float)len * 2.0 / pulse);
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (step_lr > obj_step) {
      break;
    }
  }

  stay(init_speed);
}

// 指定速度から減速して停止
void RUN::decelerate(int len, float init_speed)
{
  int obj_step;

  controlInterruptStop();
  accel = search_accel;
  speed = upper_speed_limit = init_speed;
  lower_speed_limit = MIN_SPEED;
  con_wall.enable = true;
  dirSet(MOT_FORWARD, MOT_FORWARD);
  speedSet(speed, speed);
  obj_step = (int)((float)len * 2.0 / pulse);
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (
      // 停止に必要な距離へ入ったら減速を開始
      (int)(len - step_lr_len) <
      (int)(((speed * speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * accel))) {
      break;
    }
  }

  accel = -1 * search_accel;

  while (1) {
    stepGet();
    speedSet(speed, speed);
    if (step_lr > obj_step) {
      break;
    }
  }

  stop();
}

// len[mm]の減速で止まり切れる速度(search_speed上限)。decelerate()が減速し切れず急停止するのを防ぐ
int RUN::stoppableSpeed(int len)
{
  float v_stop = sqrt(MIN_SPEED * MIN_SPEED + 2.0 * 1000.0 * search_accel * len);
  return (int)min((float)search_speed, v_stop);
}

// 指定方向へ90度単位で回転
void RUN::rotate(t_local_direction dir, int times)
{
  int obj_step;
  unsigned long start_ms = millis();
  bool timed_out = false;

  controlInterruptStop();
  accel = turn_accel;
  upper_speed_limit = search_speed;
  speed = lower_speed_limit = MIN_SPEED;
  con_wall.enable = false;
  obj_step = (int)(tread_width * PI / 4.0 * (float)times * 2.0 / pulse);

  // 左右モーターを逆方向に回してその場で旋回
  switch (dir) {
    case right:
      dirSet(MOT_FORWARD, MOT_BACK);
      break;
    case left:
      dirSet(MOT_BACK, MOT_FORWARD);
      break;
    default:
      dirSet(MOT_FORWARD, MOT_FORWARD);
      break;
  }
  speedSet(MIN_SPEED, MIN_SPEED);
  counterClear();
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if ((millis() - start_ms) > TURN_TIMEOUT_MS) {
      timed_out = true;
      break;
    }
    if (
      // 回転の残り距離から減速開始位置を判定
      (int)((tread_width * PI / 4.0 * times) - step_lr_len) <
      (int)(((speed * speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * accel))) {
      break;
    }
  }

  if (!timed_out) {
    accel = -1.0 * turn_accel;

    while (1) {
      stepGet();
      speedSet(speed_target_l, speed_target_r);
      if ((millis() - start_ms) > TURN_TIMEOUT_MS) {
        timed_out = true;
        break;
      }
      if (step_lr > obj_step) {
        break;
      }
    }
  }

  stop();

  // 基板の接触不良などでstep_lr_lenが進まず、回転が終わらないときに止める(設計メモ §8)
  if (timed_out) {
    buzzerEnable(400);
    delay(200);
    buzzerDisable();
  }
}

// 旋回の強さfを外輪の走行距離s_outから求める。外輪が全体でs_total走る間に、入口と出口を同じ傾き
// (ramp_lenで0⇔1)にした前後対称な台形にする(設計メモ §3)
static double slalomProfile(double s_out, double s_total, double ramp_len)
{
  const double f = min(1.0, min(s_out, s_total - s_out) / ramp_len);
  return max((double)SLALOM_F_MIN, f);
}

// 旋回の強さfのとき、外輪が1mm進む間に曲がる角度×tread_width
static double slalomTurnRate(double f, double inner_ratio, double outer_gain)
{
  const double outer = 1.0 + outer_gain * f;
  const double inner = 1.0 - (1.0 - inner_ratio) * f;
  return (outer - inner) / outer;
}

// fを上げる区間(f=max(F_MIN, s/ramp_len))で外輪が0→s_end(≦ramp_len)走る間の旋回角[rad]。
// 旋回開始時にも呼ぶため、f=s/ramp_lenの区間は解析的に積分する
double RUN::slalomRampAngle(double s_end)
{
  const double len = slalom_ramp_len;
  const double s_min = min(s_end, SLALOM_F_MIN * len);  // f=F_MINで一定の区間
  const double a = slalom_outer_gain + 1.0 - slalom_inner_ratio;
  const double b = slalom_outer_gain;
  // a·(s/len)/(1+b·s/len)の原始関数
  auto prim = [&](double s) {
    if (b < 1e-6) return a * s * s / (2.0 * len);
    return a / b * (s - len / b * log(1.0 + b * s / len));
  };
  double th = slalomTurnRate(SLALOM_F_MIN, slalom_inner_ratio, slalom_outer_gain) * s_min;
  if (s_end > s_min) th += prim(s_end) - prim(s_min);
  return th / tread_width;
}

// obj_th[rad]曲がるときの外輪の走行距離[mm]。台形なら旋回角が距離の一次式になるので直接求め、
// fを上げ下げする区間だけで曲がりきる三角形なら二分法で求める(設計メモ §3)
double RUN::slalomLength(double obj_th)
{
  const double len = slalom_ramp_len;
  if (obj_th >= 2.0 * slalom_ramp_th) {
    const double rate = slalomTurnRate(1.0, slalom_inner_ratio, slalom_outer_gain);
    return 2.0 * len + (obj_th - 2.0 * slalom_ramp_th) * tread_width / rate;
  }
  double lo = 0.0, hi = 2.0 * len;
  for (int i = 0; i < 30; i++) {
    const double mid = (lo + hi) / 2.0;
    if (2.0 * slalomRampAngle(mid / 2.0) < obj_th) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return hi;
}

// 停止せず外輪の増速と内輪の減速で指定角度[度]曲がる。軌跡は速度によらず、並進速度は
// slalom_speedまで下げて次の動作へつなげる。詳細は設計メモ §3参照
void RUN::slalom(t_local_direction dir, float angle)
{
  const double obj_th = (double)angle * PI / 180.0;
  const double s_total = slalomLength(obj_th);
  unsigned long start_ms = millis();
  bool timed_out = false;
  double s_out = 0.0;

  controlInterruptStop();
  con_wall.enable = false;
  turn_sign = (dir == left) ? 1 : -1;

  accel = -search_accel;
  upper_speed_limit = speed;
  lower_speed_limit = min((double)slalom_speed, (double)speed);

  turn_f = SLALOM_F_MIN;
  turn_enable = true;

  dirSet(MOT_FORWARD, MOT_FORWARD);
  counterClear();
  controlInterruptStart();

  while (1) {
    stepGetLR();
    s_out = (double)((turn_sign > 0) ? step_r : step_l) * pulse;
    const double th = fabs((double)(step_r - step_l)) * pulse / tread_width;
    // s_totalを過ぎても角度が足りなければF_MINで回り続ける
    turn_f = slalomProfile(s_out, s_total, slalom_ramp_len);
    speedSet(speed_target_l, speed_target_r);
    if ((millis() - start_ms) > TURN_TIMEOUT_MS) {
      timed_out = true;
      break;
    }
    if (th >= obj_th) {
      break;
    }
  }

  turn_enable = false;
  turn_f = 0.0;
  accel = 0.0;
  slalom_tail = (float)max(0.0, s_out - s_total);

  // 基板の接触不良等でパルス数が進まず旋回が終わらない場合は強制停止する(設計メモ §8)
  if (timed_out) {
    upper_speed_limit = lower_speed_limit = MIN_SPEED;
    stop();
    buzzerEnable(400);
    delay(200);
    buzzerDisable();
  } else {
    upper_speed_limit = lower_speed_limit = speed;
  }

  // straight()等は開始時にcounterClear()しない前提のため、ここで明示的にクリアする(設計メモ §2)
  counterClear();
}

// 区画境界から次の区画境界までを1動作で90度曲がる(設計メモ §3)
void RUN::slalomCell(t_local_direction dir)
{
  const int pre = max(0, (int)(HALF_SECTION - slalom_offset + 0.5f));
  if (pre > 0) oneStep(pre, (int)speed);
  slalom(dir, 90);
  if (pre > 0) oneStep(pre, (int)speed);
}

// スラロームの形のパラメータを範囲内に収め、90度旋回の張り出しslalom_offsetを、slalomProfile()の
// fを数値積分して求める(設計メモ §3)
void RUN::slalomParamUpdate(void)
{
  if ((slalom_inner_ratio < 0.0f) || (slalom_inner_ratio > SLALOM_INNER_RATIO_MAX)) {
    slalom_inner_ratio = SLALOM_INNER_RATIO_INIT;
  }
  if ((slalom_outer_gain < 0.0f) || (slalom_outer_gain > SLALOM_OUTER_GAIN_MAX)) {
    slalom_outer_gain = SLALOM_OUTER_GAIN_INIT;
  }
  if ((slalom_ramp_len < 1.0f) || (slalom_ramp_len > SLALOM_RAMP_LEN_MAX)) {
    slalom_ramp_len = SLALOM_RAMP_LEN_INIT;
  }

  slalom_ramp_th = (float)slalomRampAngle(slalom_ramp_len);

  const double obj_th = PI / 2.0;
  const double s_total = slalomLength(obj_th);
  const double ds = 0.1;  // 外輪の走行距離の刻み[mm]
  double th = 0.0, s_out = 0.0, fwd = 0.0, side = 0.0;
  while ((th < obj_th) && (s_out < 1000.0)) {
    const double f = slalomProfile(s_out + ds / 2.0, s_total, slalom_ramp_len);
    const double outer = 1.0 + slalom_outer_gain * f;
    const double inner = 1.0 - (1.0 - slalom_inner_ratio) * f;
    const double dth = (outer - inner) / outer * ds / tread_width;
    const double dc = (outer + inner) / (2.0 * outer) * ds;
    fwd += dc * cos(th + dth / 2.0);
    side += dc * sin(th + dth / 2.0);
    th += dth;
    s_out += ds;
  }
  slalom_offset = (float)fwd;
  // 前後対称なのでfwdとsideは一致する(確認用に両方出す)
  Serial.printf(
    "slalom_offset = %.1f mm (side %.1f mm, len %.1f mm)\r\n", slalom_offset, side, s_total);
  if (slalom_offset > HALF_SECTION) {
    Serial.println("warning: slalom_offset exceeds HALF_SECTION");
  }
}

// 指定方向(前進/後退)へ一定速度で指定距離だけ動いて停止する
// (Uターンの位置補正など、小さな直線動作用。呼び出し前に停止していることが前提)
void RUN::moveFixed(t_CW_CCW dir, int len, int spd)
{
  int obj_step;

  controlInterruptStop();
  accel = 0.0;
  speed = lower_speed_limit = upper_speed_limit = (double)spd;
  con_wall.enable = false;
  dirSet(dir, dir);
  speedSet(spd, spd);
  counterClear();
  obj_step = (int)((float)len * 2.0 / pulse);
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (step_lr > obj_step) {
      break;
    }
  }

  stop();
}

// 行き止まりUターン: (90度旋回->バック->前進)×2で180度回転し、縦横の位置を中心へ補正する。
// rotate(right,2)の代わりに使う。前後の減速・加速は呼び出し側が行う
void RUN::uTurn(t_local_direction dir)
{
  rotate(dir, 1);
  moveFixed(MOT_BACK, uturn_back_len, uturn_speed);
  moveFixed(MOT_FORWARD, wall_return_len, uturn_speed);
  rotate(dir, 1);
  moveFixed(MOT_BACK, uturn_back_len, uturn_speed);
  moveFixed(MOT_FORWARD, wall_return_len, uturn_speed);
}