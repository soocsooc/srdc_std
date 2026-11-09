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

typedef enum { PARAM_SHORT, PARAM_FLOAT, PARAM_DOUBLE } t_param_type;

// /parameters.txtの1行(キーと値)。値がmin〜maxの外ならinitへ戻す(min>maxなら範囲チェックなし)。
// 範囲チェックの理由は設計メモ §12
typedef struct
{
  const char * key;
  t_param_type type;
  void * ptr;
  float init;
  float min;
  float max;
} t_param_entry;

#define NO_CHECK 1.0f, 0.0f
#define SPEED_RANGE MIN_SPEED, SPEED_SANITY_MAX
#define ACCEL_RANGE 0.001f, ACCEL_SANITY_MAX

static const t_param_entry param_table[] = {
  {"ref_sen_r", PARAM_SHORT, &g_sensor.base.ref_r, REF_SEN_R_INIT, NO_CHECK},
  {"ref_sen_l", PARAM_SHORT, &g_sensor.base.ref_l, REF_SEN_L_INIT, NO_CHECK},
  {"th_sen_r", PARAM_SHORT, &g_sensor.base.th_r, TH_SEN_R_INIT, NO_CHECK},
  {"th_sen_l", PARAM_SHORT, &g_sensor.base.th_l, TH_SEN_L_INIT, NO_CHECK},
  {"th_sen_f", PARAM_SHORT, &g_sensor.base.th_f, TH_SEN_F_INIT, NO_CHECK},
  {"sen_f_wall", PARAM_SHORT, &g_sensor.base.f_wall, SEN_F_WALL_INIT, NO_CHECK},
  {"sen_f_nowall", PARAM_SHORT, &g_sensor.base.f_nowall, SEN_F_NOWALL_INIT, NO_CHECK},
  {"th_sen_f_approach", PARAM_SHORT, &g_sensor.base.th_approach, TH_SEN_F_APPROACH_INIT,
   NO_CHECK},
  {"th_sen_f_approach_slalom", PARAM_SHORT, &g_sensor.base.th_approach_slalom,
   TH_SEN_F_APPROACH_SLALOM_INIT, NO_CHECK},
  {"th_sen_r_slalom", PARAM_SHORT, &g_sensor.base.th_r_slalom, TH_SEN_R_INIT, NO_CHECK},
  {"th_sen_l_slalom", PARAM_SHORT, &g_sensor.base.th_l_slalom, TH_SEN_L_INIT, NO_CHECK},
  {"sen_ratio", PARAM_SHORT, &g_sensor.ratio_pct, SEN_RATIO_INIT, SEN_RATIO_MIN, SEN_RATIO_MAX},
  {"maze_x", PARAM_SHORT, &g_map.maze_x, MAZESIZE_X_INIT, NO_CHECK},
  {"maze_y", PARAM_SHORT, &g_map.maze_y, MAZESIZE_Y_INIT, NO_CHECK},
  {"goal_x", PARAM_SHORT, &g_map.goal_mx, GOAL_X_INIT, NO_CHECK},
  {"goal_y", PARAM_SHORT, &g_map.goal_my, GOAL_Y_INIT, NO_CHECK},
  {"goal_x2", PARAM_SHORT, &g_map.goal_mx2, GOAL_X2_INIT, NO_CHECK},
  {"goal_y2", PARAM_SHORT, &g_map.goal_my2, GOAL_Y2_INIT, NO_CHECK},
  {"tire_d", PARAM_FLOAT, &g_run.tire_diameter, TIRE_DIAMETER_INIT, NO_CHECK},
  {"tread_w", PARAM_FLOAT, &g_run.tread_width, TREAD_WIDTH_INIT, NO_CHECK},
  {"con_kp", PARAM_DOUBLE, &g_run.con_wall.kp, CON_WALL_KP_INIT, NO_CHECK},
  {"search_accel", PARAM_FLOAT, &g_run.search_accel, SEARCH_ACCEL_INIT, ACCEL_RANGE},
  {"turn_accel", PARAM_FLOAT, &g_run.turn_accel, TURN_ACCEL_INIT, ACCEL_RANGE},
  {"search_speed", PARAM_SHORT, &g_run.search_speed, SEARCH_SPEED_INIT, SPEED_RANGE},
  {"max_speed", PARAM_SHORT, &g_run.max_speed, MAX_SPEED_INIT, SPEED_RANGE},
  {"slalom_speed", PARAM_SHORT, &g_run.slalom_speed, SLALOM_SPEED_INIT, SPEED_RANGE},
  {"slalom_inner_ratio", PARAM_FLOAT, &g_run.slalom_inner_ratio, SLALOM_INNER_RATIO_INIT, 0.0f,
   SLALOM_INNER_RATIO_MAX},
  {"slalom_outer_gain", PARAM_FLOAT, &g_run.slalom_outer_gain, SLALOM_OUTER_GAIN_INIT, 0.0f,
   SLALOM_OUTER_GAIN_MAX},
  {"slalom_ramp_len", PARAM_FLOAT, &g_run.slalom_ramp_len, SLALOM_RAMP_LEN_INIT, 1.0f,
   SLALOM_RAMP_LEN_MAX},
  {"uturn_back_len", PARAM_SHORT, &g_run.uturn_back_len, UTURN_BACK_LEN_INIT, 1, SECTION},
  {"wall_return_len", PARAM_SHORT, &g_run.wall_return_len, WALL_RETURN_LEN_INIT, 1, SECTION},
  {"uturn_speed", PARAM_SHORT, &g_run.uturn_speed, UTURN_SPEED_INIT, SPEED_RANGE},
};
static const int PARAM_COUNT = sizeof(param_table) / sizeof(param_table[0]);

static void paramStore(const t_param_entry & e, float v)
{
  switch (e.type) {
    case PARAM_SHORT:
      *(short *)e.ptr = (short)lroundf(v);
      break;
    case PARAM_FLOAT:
      *(float *)e.ptr = v;
      break;
    case PARAM_DOUBLE:
      *(double *)e.ptr = (double)v;
      break;
  }
}

static String paramFormat(const t_param_entry & e)
{
  switch (e.type) {
    case PARAM_SHORT:
      return String(*(short *)e.ptr);
    case PARAM_FLOAT:
      return String(*(float *)e.ptr, 3);
    case PARAM_DOUBLE:
      return String(*(double *)e.ptr, 3);
  }
  return "";
}

// 全パラメータを初期値にする
static void paramDefault(void)
{
  for (int i = 0; i < PARAM_COUNT; i++) paramStore(param_table[i], param_table[i].init);
}

int paramCount(void) { return PARAM_COUNT; }

const char * paramKey(int index) { return param_table[index].key; }

// キーに対応するパラメータへ値を設定する。範囲外なら初期値に戻す。未知のキーはfalse
bool paramSet(const String & key, const String & value)
{
  for (int i = 0; i < PARAM_COUNT; i++) {
    const t_param_entry & e = param_table[i];
    if (!key.equals(e.key)) continue;
    float v = value.toFloat();
    if ((e.min <= e.max) && ((v < e.min) || (v > e.max))) {
      Serial.printf("parameter %s invalid (%f) -> fallback to init\r\n", e.key, v);
      v = e.init;
    }
    paramStore(e, v);
    return true;
  }
  return false;
}

// パラメータから決まる値(1パルスの距離・センサの判定値・スラロームの張り出し)を作り直す
void paramApply(void)
{
  g_run.pulse = g_run.tire_diameter * PI / PULSE_PER_REV;
  g_sensor.applyRatio();
  g_run.slalomParamUpdate();
}

// paramSet()で値を変えた後に呼ぶ。迷路の大きさとゴールを検証し、大きさが変わっていたら迷路情報を
// 消す。パラメータから決まる値を作り直してFlashへ保存する(Web/シリアル共通)
void paramCommit(short old_maze_x, short old_maze_y)
{
  // 迷路の大きさが変わると外周の壁も変わるため、迷路情報を消去する(設計メモ §9)
  g_map.mazeSizeValidate();
  if ((g_map.maze_x != old_maze_x) || (g_map.maze_y != old_maze_y)) {
    mapErase();
  }
  g_map.goalValidate();
  paramApply();
  paramWrite();
}

// 全パラメータを「key value」形式でシリアルへ出力
void paramPrint(void)
{
  for (int i = 0; i < PARAM_COUNT; i++) {
    Serial.printf("%s %s\r\n", param_table[i].key, paramFormat(param_table[i]).c_str());
  }
}

static void paramSerialExec(String line)
{
  line.trim();
  if (line.length() == 0) return;
  int sp = line.indexOf(' ');
  String cmd = (sp < 0) ? line : line.substring(0, sp);
  String args = (sp < 0) ? "" : line.substring(sp + 1);
  args.trim();

  if (cmd.equals("list")) {
    paramPrint();
    return;
  }
  if (cmd.equals("list_r")) {
    g_misc.modeListPrint();
    return;
  }
  if (cmd.equals("list_a")) {
    g_adjust.modeListPrint();
    return;
  }
  if (cmd.equals("set")) {
    int sp2 = args.indexOf(' ');
    if (sp2 < 0) {
      Serial.println("usage: set <key> <value>");
      return;
    }
    String key = args.substring(0, sp2);
    String value = args.substring(sp2 + 1);
    value.trim();
    const short old_maze_x = g_map.maze_x, old_maze_y = g_map.maze_y;
    if (!paramSet(key, value)) {
      Serial.printf("unknown key: %s\r\n", key.c_str());
      return;
    }
    paramCommit(old_maze_x, old_maze_y);
    for (int i = 0; i < PARAM_COUNT; i++) {
      if (key.equals(param_table[i].key)) {
        Serial.printf("%s = %s\r\n", param_table[i].key, paramFormat(param_table[i]).c_str());
      }
    }
    buzzerEnable(INC_FREQ);
    delay(30);
    buzzerDisable();
    return;
  }
  Serial.println("commands: list | list_r | list_a | set <key> <value>");
}

// USBシリアルからのパラメータ設定コマンドを受け付ける(ブロックしない)
void paramSerialPoll(void)
{
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if ((c == '\r') || (c == '\n')) {
      paramSerialExec(buf);
      buf = "";
    } else if (buf.length() < 64) {
      buf += c;
    }
  }
}

// 全パラメータを初期値に戻し、/parameters.txtへ書き込む
void flashInit(void)
{
  Serial.println("\n\r parameter init /parameters.txt");
  paramDefault();
  paramWrite();
}

// SPIFFSを初期化し、必要ならパラメータを初期化
void flashBegin(void)
{
  controlInterruptStop();
  sensorInterruptStop();
  delay(100);

  Serial.println("begin SPIFFS");
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    delay(100);
  }
  Serial.println("end SPIFFS");

  if (switchGet() == SW_CM) {
    delay(10);
    flashInit();
  }

  controlInterruptStart();
  sensorInterruptStart();
}

// 迷路の壁情報をバイナリ形式でSPIFFSへ保存
void mapWrite(void)
{
  String file_tmp;
  unsigned char data_temp;
  file_tmp = "/map.txt";

  controlInterruptStop();
  sensorInterruptStop();

  Serial.printf("\n\r map_data write ");
  Serial.println(file_tmp);

  File file = SPIFFS.open(file_tmp, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    controlInterruptStart();  // 早期returnで割り込みを止めたままにしない(設計メモ §10)
    sensorInterruptStart();
    return;
  }
  // ファイルは16x16=256バイトのまま、実際の迷路範囲(maze_x/maze_y)外は0を書く(範囲外参照の防止、設計メモ §10)
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      if ((i >= g_map.maze_x) || (j >= g_map.maze_y)) {
        data_temp = 0;
      } else {
        data_temp = g_map.wallDataGet(i, j, north) + (g_map.wallDataGet(i, j, east) << 2) +
                    (g_map.wallDataGet(i, j, south) << 4) + (g_map.wallDataGet(i, j, west) << 6);
      }
      if (file.write(data_temp)) {
      } else {
        Serial.println("- write failed");
      }
    }
  }

  file.close();
  controlInterruptStart();
  sensorInterruptStart();
}

// 壁情報を未知の状態へ戻して/map.txtへ書き戻す(走行モード6用)。詳細は設計メモ §10参照
void mapErase(void)
{
  g_map.mapClear();
  mapWrite();
}

// SPIFFSから迷路の壁情報を読み込む
void mapCopy(void)
{
  String file_tmp;
  unsigned char read_data;

  controlInterruptStop();
  sensorInterruptStop();

  File file = SPIFFS.open("/map.txt", FILE_READ);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    controlInterruptStart();  // 早期returnで割り込みを止めたままにしない(設計メモ §10)
    sensorInterruptStart();
    return;
  }
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      if (file.available()) {
        read_data = file.read();
        if ((i >= g_map.maze_x) || (j >= g_map.maze_y)) continue;  // 範囲外へ書かない(設計メモ §10)
        g_map.wallDataSet(i, j, north, read_data & 0x03);
        g_map.wallDataSet(i, j, east, (read_data >> 2) & 0x03);
        g_map.wallDataSet(i, j, south, (read_data >> 4) & 0x03);
        g_map.wallDataSet(i, j, west, (read_data >> 6) & 0x03);
      } else {
        Serial.println("Read Error");
      }
    }
  }
  file.close();
  controlInterruptStart();
  sensorInterruptStart();
}

// 現在のパラメータをファイルへ保存
void paramWrite(void)
{
  String text;
  for (int i = 0; i < PARAM_COUNT; i++) {
    text += String(param_table[i].key) + ' ' + paramFormat(param_table[i]) + '\n';
  }

  //タイマーによる割り込みは禁止にする必要がある。
  controlInterruptStop();
  sensorInterruptStop();
  delay(10);
  writeFile(SPIFFS, "/parameters.txt", text);
  controlInterruptStart();
  sensorInterruptStart();
}

// パラメータファイルを読み込み、各モジュールへ反映
void paramRead(void)
{
  String cmds[2] = {"\0"};  // 分割された文字列を格納する配列

  //タイマーによる割り込みは禁止にする必要がある。
  controlInterruptStop();
  sensorInterruptStop();
  delay(10);

  // ファイルに無いキーは初期値のまま使う(古いparameters.txt対策、設計メモ §12)
  paramDefault();
  // goal_x2/y2が無い古いファイルでは1区画ゴール(goal_x/yと同じ)として扱う(設計メモ §9)
  g_map.goal_mx2 = g_map.goal_my2 = -1;

  File file = SPIFFS.open("/parameters.txt");
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    flashInit();
  } else {
    Serial.println("- read from file:");
    while (file.available()) {
      split(file.readStringUntil('\n'), ' ', cmds);
      // 削除したパラメータのキーは読み飛ばす(設計メモ §12)
      if (!paramSet(cmds[0], cmds[1])) {
        Serial.print("parameter unknown key (ignored) ");
        Serial.println(cmds[0]);
      }
      cmds[0] = "";
      cmds[1] = "";
    }
    file.close();
  }

  if (g_map.goal_mx2 < 0) g_map.goal_mx2 = g_map.goal_mx;
  if (g_map.goal_my2 < 0) g_map.goal_my2 = g_map.goal_my;
  g_map.mazeSizeValidate();
  g_map.mapClear();  // 起動時の壁情報(外周)を読み込んだ迷路の大きさで作り直す(設計メモ §9)
  g_map.goalValidate();
  paramApply();
  controlInterruptStart();
  sensorInterruptStart();
}

int split(String data, char delimiter, String * dst)
{
  int index = 0;
  int arraySize = (sizeof(data) / sizeof((data)[0]));
  int datalength = data.length();
  for (int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    if (tmp == delimiter) {
      index++;
      if (index > (arraySize - 1)) return -1;
    } else
      dst[index] += tmp;
  }
  return (index + 1);
}

void writeFile(fs::FS & fs, String path, String message)
{
  Serial.printf("Writing file: ");
  Serial.println(path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
  } else {
    Serial.println("- frite failed");
  }
  file.close();
}

void appendFile(fs::FS & fs, String path, String message)
{
  Serial.printf("Appending to file: ");
  Serial.println(path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
  } else {
    Serial.println("- append failed");
  }
  file.close();
}
