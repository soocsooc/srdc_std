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

ADJUST g_adjust;

// センサ値とバッテリー電圧をシリアルモニタに表示
// いずれかのスイッチを押すと表示を終了して呼び出し元へ戻る
void ADJUST::adcView(void)
{
  motorDisable();

  while (switchGet() == 0) {
    Serial.printf("r_sen        is\t%d   \r\n", g_sensor.sen_r.value);
    Serial.printf("f_sen        is\t%d   \r\n", g_sensor.sen_f.value);
    Serial.printf("l_sen        is\t%d   \r\n", g_sensor.sen_l.value);
    Serial.printf("VDD          is\t%d mV\r\n", g_sensor.battery_value);
    // SLED点灯窓が伸びて棄却したサンプルの累計 (前/右/左)。WiFi接続中に増える。
    Serial.printf(
      "reject F/R/L\t%lu %lu %lu\r\n", g_sensor.sample_reject[0], g_sensor.sample_reject[1],
      g_sensor.sample_reject[2]);
    // SLED点灯窓[us]の最大値 (前/右/左)。左3つは前回表示から、max()は起動からの最大。
    // 区間最大を0に戻す間にsensorTaskが書いた1サンプルは失うことがあるが、確認用なので許容する
    Serial.printf(
      "on_us F/R/L\t%u %u %u  max(%u %u %u)\r\n", g_sensor.sled_on_us_max[0],
      g_sensor.sled_on_us_max[1], g_sensor.sled_on_us_max[2], g_sensor.sled_on_us_max_all[0],
      g_sensor.sled_on_us_max_all[1], g_sensor.sled_on_us_max_all[2]);
    for (int i = 0; i < 3; i++) g_sensor.sled_on_us_max[i] = 0;
    Serial.printf("core timer0/sensor\t%d %d\r\n", g_core_timer0, g_core_sensor);
    Serial.printf("\n\r");
    delay(100);
    Serial.printf("\x1b[2J");
    Serial.printf("\x1b[0;0H");
  }
}

// 指定区間数を直進して走行を確認
void ADJUST::straightCheck(int section_check)
{
  motorEnable();
  delay(1000);
  g_run.startAccelerate(HALF_SECTION, g_run.search_speed);
  if (section_check > 1) {
    g_run.straight(
      SECTION * (section_check - 1), g_run.search_speed, g_run.max_speed, g_run.search_speed);
  }
  g_run.decelerate(HALF_SECTION, g_run.search_speed);

  motorDisable();
}

// 90度回転を繰り返して回転動作を確認
void ADJUST::rotationCheck(void)
{
  motorEnable();
  delay(1000);
  for (int i = 0; i < 8; i++) {
    g_run.rotate(right, 1);
  }
  motorDisable();
}

// slalom()がs_totalを超えて余計に走った距離をLEDの点灯数で表す(5mm未満で1個、45mm以上で4個)
static unsigned char slalomTailLed(float tail)
{
  if (tail < 5.0f) return 0b0001;
  if (tail < 20.0f) return 0b0011;
  if (tail < 45.0f) return 0b0111;
  return 0b1111;
}

// 4x4区画コースの最外周をスラローム旋回でつないで周回する。各辺は2区画の直進+角の
// slalomCell()。switchGet()の判定は角を曲がった後の区画境界でのみ行う(設計メモ §3)。
// 角を曲がった後は、その旋回の余計な走行距離をslalomTailLed()で次の角まで表示する(設計メモ §3)
void ADJUST::circuitCheck(void)
{
  motorEnable();
  delay(1000);

  ledSet(0b0000);
  g_run.startAccelerate(HALF_SECTION, g_run.search_speed);

  do {
    g_run.straight(SECTION * 2, g_run.speed, g_run.search_speed, g_run.slalom_speed);
    g_run.slalomCell(right);
    ledSet(slalomTailLed(g_run.slalom_tail));
  } while (switchGet() == 0);

  g_run.decelerate(HALF_SECTION, g_run.speed);
  motorDisable();
}

// 連続旋回(S字)・旋回直後の再加速・U字(同方向の連続旋回)を一続きに確認する。
// スタートから 右→左→1区画直進→右→右 の順に曲がれるコースで行う(設計メモ §3)。
void ADJUST::slalomSequenceCheck(void)
{
  motorEnable();
  delay(1000);

  ledSet(0b0001);
  g_run.startAccelerate(HALF_SECTION, g_run.slalom_speed);

  // 連続旋回(S字): 区画境界ごとにslalomCell()をつなぐ
  ledSet(0b0010);
  g_run.slalomCell(right);
  g_run.slalomCell(left);

  // 旋回直後の直進
  ledSet(0b0001);
  g_run.straight(SECTION, g_run.speed, g_run.search_speed, g_run.slalom_speed);

  // U字: 同方向のslalomCell()を2回つなぐ
  ledSet(0b0100);
  g_run.slalomCell(right);
  g_run.slalomCell(right);

  ledSet(0b0001);
  g_run.decelerate(HALF_SECTION, g_run.speed);

  motorDisable();
}

// 到達判定閾値の実測用。センサ停止を無効化(INT_MAX)して壁へ向けて走り、走行距離と前センサ値を
// Serialへ出力する。スイッチで即停止できる。詳細は設計メモ §4参照。
void ADJUST::straightSensorDiag(float section_check)	// 最大走行区画数(壁の手前で止まる値にすること)
{
  motorEnable();
  delay(1000);
  g_run.startAccelerate(HALF_SECTION, g_run.search_speed);
  if (!g_run.oneStepDynamic((int)(SECTION * section_check), g_run.search_speed, INT_MAX, true)) {
    g_run.decelerate(HALF_SECTION, g_run.speed);
  }
  motorDisable();
}

// 前壁が見えるまで直進し、区画中心で止まる(Pゲイン調整用)。モード2と同じくth_approachで
// 区画境界とみなしてdecelerate(HALF_SECTION)する。走行中はログを出し、スイッチで即停止できる(STEP8_動作まとめ.md §5)
void ADJUST::straightToWallCheck(void)
{
  const int cruise_speed = g_run.stoppableSpeed(HALF_SECTION);  // 半区画で止まり切れる速度

  motorEnable();
  delay(1000);
  g_run.startAccelerate(HALF_SECTION, cruise_speed);
  if (!g_run.oneStepDynamic(
        SECTION * MAZESIZE_MAX, cruise_speed, g_sensor.th_approach, true)) {
    g_run.decelerate(HALF_SECTION, g_run.speed);
  }
  motorDisable();
}

// 単発のslalomCell()の確認(モード3と同じ入り方)。スタートから2区画直進した区画(前壁あり)で
// 曲がり、次の区画の中心で止まる。張り出しslalom_offsetをSerialへ出力する(設計メモ §3)。
void ADJUST::slalomTurnCheck(t_local_direction dir)
{
  Serial.printf("slalom_offset = %.1f mm\r\n", g_run.slalom_offset);
  motorEnable();
  delay(1000);
  g_run.startAccelerate(HALF_SECTION, g_run.search_speed);
  g_run.oneStepDynamic(SECTION * 2, g_run.search_speed, g_sensor.th_approach_slalom);
  g_run.slalomCell(dir);
  g_run.decelerate(HALF_SECTION, g_run.speed);
  motorDisable();
}

// 「決定」を待って前センサを約1秒平均し、ベース値(Sensor ratio=100%相当)で返す。「次」ならfalse
static bool frontSensorSample(short * f)
{
  const int samples = 100;
  unsigned char sw;
  while ((sw = switchGet()) == 0) delay(10);
  if (sw != SW_CM) return false;
  delay(500);  // スイッチを押した手がセンサにかからないよう待つ

  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += g_sensor.sen_f.value;
    delay(10);
  }
  const long div = (long)samples * g_sensor.ratio_pct / 100;
  *f = (short)((sum + div / 2) / div);
  return true;
}

// 区画中央に置き、1区画先の前壁あり→なしで前センサを測り、測定値と平均(th_sen_f)を保存する。
// LEDは前壁あり=0b0110、なし=0b1001。「次」で中止。詳細は設計メモ §7参照
void ADJUST::frontThresholdCalib(void)
{
  short f_wall, f_nowall;

  Serial.println("place at cell center, wall 1 cell ahead (no wall in this cell) -> center SW");
  ledSet(0b0110);
  if (!frontSensorSample(&f_wall)) return;
  Serial.printf("wall    f=%d\r\n", f_wall);
  g_misc.buttonOk();

  Serial.println("remove the wall -> center SW");
  ledSet(0b1001);
  if (!frontSensorSample(&f_nowall)) return;
  Serial.printf("no wall f=%d\r\n", f_nowall);

  t_sensor_base & b = g_sensor.base;
  b.f_wall = f_wall;
  b.f_nowall = f_nowall;
  b.th_f = (f_wall + f_nowall) / 2;
  paramApply();
  paramWrite();
  Serial.printf("th_sen_f=%d (base)\r\n", b.th_f);
  g_misc.buttonOk();
}

// 調整メニュー。LED3を点滅させながら「次」で番号を選び、「決定」でmodeExec()を実行する。
// modeExec()が1を返したら(11を選んだら)走行モードの選択へ戻る
void ADJUST::menu(void)
{
  unsigned char l_mode = 1;
  char LED3_data;
  char sw;

  while (1) {
    ledSet(l_mode);
    while (1) {
      sw = switchGet();
      if (sw != 0) break;
      paramSerialPoll();  // 待機中はUSBシリアルからのパラメータ設定を受け付ける
      delay(33);
      LED3_data ^= 1;
      ledSet((l_mode & 0x7) + ((LED3_data << 3) & 0x08));
    }
    LED3_data = 0;
    switch (sw) {
      case SW_RM:
        l_mode = g_misc.buttonInc(l_mode, 11, 1);
        break;
      case SW_CM:
        g_misc.buttonOk();
        if (modeExec(l_mode) == 1) {
          return;
        }
        break;
      default:
        break;
    }
  }
}

// 調整モードの一覧をシリアルへ出力する(USBシリアルのlist_a)。modeExec()のcaseと対応させること
void ADJUST::modeListPrint(void)
{
  static const char * const names[] = {
    "1: web server (192.168.4.1)",
    "2: rotation check (tread_w)",
    "3: circuit check (4x4 slalom lap)",
    "4: straight to front wall (con_kp)",
    "5: straight 3 sections (tire_d)",
    "6: adc view (serial)",
    "7: slalom sequence check",
    "8: straight sensor diag",
    "9: slalom turn check (right)",
    "10: front threshold calib (center, wall 1 cell ahead -> no wall)",
    "11: exit to main menu",
  };
  for (const char * n : names) Serial.println(n);
}

// 選択された調整項目を実行
unsigned char ADJUST::modeExec(unsigned char l_mode)
{
  motorDisable();
  switch (l_mode) {
    case 1:
      buzzerEnable(INC_FREQ);
      delay(30);
      buzzerDisable();
      delay(500);
      buzzerEnable(INC_FREQ);
      delay(30);
      buzzerDisable();
      webServerSetup(); // センサ値等を確認するためのWebサーバをセットアップ (192.168.4.1)
      //webserverでセンサの値のばらつきが大きすぎて判断できない時は、調整モード6(adcView)でシリアルモニタから確認
      break;
    case 2:
      rotationCheck();	// タイヤ間距離を調整、旋回角度が足りなければ距離を大きく
      break;
    case 3:
      circuitCheck();		// 4x4コース最外周をスラローム周回で確認
      break;
		case 4:
      straightToWallCheck();  // 前壁まで直進してPゲイン調整、壁際に置いて3~4区画で中心が目標
      break;
    case 5:
			straightCheck(3);	// 3区画前進で直径を調整、目標より手前だったら直径を小さく
      break;
    case 6:
      // 調整モード1のadcView版: シリアルモニタでセンサ値/電池電圧を確認 (いずれかのスイッチで調整メニューへ戻る)
      buzzerEnable(INC_FREQ);
      delay(30);
      buzzerDisable();
      delay(500);
      buzzerEnable(INC_FREQ);
      delay(30);
      buzzerDisable();
      adcView();
      break;
    case 7:
      slalomSequenceCheck();	// 連続旋回・行き止まりUターンを含めた一連の動作を確認
      break;
    case 8:
      // th_approach決定用。壁まで580mmの環境で余裕を取った値(STEP8_動作まとめ.md §5参照)。
      // 壁の距離を変えたらここも変更する
      straightSensorDiag(2.2);
      break;
    case 9:
      slalomTurnCheck(right);  // 単発の旋回の確認(左を確認するときはleftへ変更)
      break;
    case 10:
      frontThresholdCalib();  // 1区画先の前壁あり/なしを測り、平均を前センサ閾値にする
      break;
    default:
      return 1;
      break;
  }

  return 0;
}
