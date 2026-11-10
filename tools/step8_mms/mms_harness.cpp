// STEP8 の迷路ロジック(map_manager/search/search_dynamic/fast/misc の .cpp)を実機のまま、
// マイクロマウスシミュレーター mms(https://github.com/mackorone/mms)の上で動かす。
// RUN の動作は「真の位置[mm]と向きを動かし、mms のマウスを最も近い半区画の格子点へ動かす」に置き換え、
// 壁センサーの値は mms に問い合わせる。使い方は README.md 参照
#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <unistd.h>  // dup, dup2

#include "sim_shim.h"  // ビルドでは -include でも読むが、エディタの解析用に明示する

unsigned long g_sim_ms;
bool g_sim_verbose;
SimSerial Serial;

RUN g_run;
SENSOR g_sensor;
ADJUST g_adjust;

// ---- mms との通信 ----
// stdout は mms へのコマンド専用。Serial や printf の出力が混ざらないよう、起動時に元の stdout を
// 複製して g_mms に移し、stdout を stderr へつなぎ替える(stderr は mms のログ欄に出る)
static FILE * g_mms;

static std::string mmsReadLine(void)
{
  char buf[256];
  if (!fgets(buf, sizeof buf, stdin)) {
    fprintf(stderr, "mms: stdin closed\n");
    exit(1);
  }
  std::string s(buf);
  while (!s.empty() && ((s.back() == '\n') || (s.back() == '\r'))) s.pop_back();
  return s;
}
static void mmsSend(const char * fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vfprintf(g_mms, fmt, ap);
  va_end(ap);
  fputc('\n', g_mms);
  fflush(g_mms);
}
static std::string mmsQuery(const char * cmd)
{
  mmsSend("%s", cmd);
  return mmsReadLine();
}
static bool mmsBool(const char * cmd) { return mmsQuery(cmd) == "true"; }

// ---- 中断(失敗・mms のリセットボタン) ----
struct SimAbort
{
  std::string why;
};
struct SimReset
{
};

// ---- ハードウェアの置き換え ----
void ledSet(unsigned char) {}
void buzzerEnable(short) {}
void buzzerDisable(void) {}
void motorEnable(void) {}
void motorDisable(void) {}
unsigned char switchGet(void) { return 1; }  // 到達不能の点滅ループを即座に抜ける
void ADJUST::menu(void) {}

// SPIFFS の /map.txt の代わり(実機の mapWrite/mapCopy と同じ 16x16 バイトの形式)
static unsigned char flash_map[16][16];
void mapWrite(void)
{
  for (int i = 0; i < 16; i++)
    for (int j = 0; j < 16; j++)
      flash_map[i][j] = ((i >= g_map.maze_x) || (j >= g_map.maze_y))
                          ? 0
                          : g_map.wallDataGet(i, j, north) + (g_map.wallDataGet(i, j, east) << 2) +
                              (g_map.wallDataGet(i, j, south) << 4) +
                              (g_map.wallDataGet(i, j, west) << 6);
}
void mapCopy(void)
{
  for (int i = 0; i < g_map.maze_x; i++)
    for (int j = 0; j < g_map.maze_y; j++) {
      const unsigned char d = flash_map[i][j];
      g_map.wallDataSet(i, j, north, d & 0x03);
      g_map.wallDataSet(i, j, east, (d >> 2) & 0x03);
      g_map.wallDataSet(i, j, south, (d >> 4) & 0x03);
      g_map.wallDataSet(i, j, west, (d >> 6) & 0x03);
    }
}
void mapErase(void)
{
  g_map.mapClear();
  mapWrite();
}

// ---- 真の姿勢と mms のマウスの位置 ----
static const int DX[4] = {0, 1, 0, -1}, DY[4] = {1, 0, -1, 0};  // north, east, south, west
static const char * const DIR_NAME[4] = {"N", "E", "S", "W"};
static const char DIR_CHAR[4] = {'n', 'e', 's', 'w'};
static int W, H, GX, GY, GX2, GY2;

// 真の姿勢。位置は mm で、区画 (cx,cy) の中心が (cx*SECTION+HALF_SECTION, ...)。
// (hx,hy) は mms のマウスの位置で、単位は半区画(区画 (0,0) の中心が (1,1)、区画の境目は偶数)
static struct
{
  int x, y, dir;
  int hx, hy;
  long calls, call_limit;
} pose;

// モードの比較用に、往路(モード開始からゴールの区画に初めて入るまで)を記録する
struct OutboundLog
{
  bool active;     // 往路の途中
  long mm;         // 走った距離[mm]
  int pivot;       // 超信地旋回(rotate・uTurn)の回数。90度を 1 回と数える
  int slalom;      // slalomCell() の回数
  long est_ms;     // fastPathMake() の推定時間[ms](Serial の "est." から読む)。なければ -1
  std::vector<int> route;  // fastPathMake() の経路が通る区画(x*MAZESIZE_MAX+y)。最短走行だけ
};
static OutboundLog g_out;

static bool inGoalCell(int cx, int cy)
{
  return (cx >= min(GX, GX2)) && (cx <= max(GX, GX2)) && (cy >= min(GY, GY2)) &&
         (cy <= max(GY, GY2));
}
static bool inGoal(int x_mm, int y_mm) { return inGoalCell(x_mm / SECTION, y_mm / SECTION); }

// ---- 最短走行の経路の色付け ----
// モード 4 はオレンジ、モード 5 は水色。全モードの後の重ね表示では、両方が通る区画を黄色にする。
// ゴールの区画は濃い緑のまま変えない
static const char ROUTE_COLOR_4 = 'o', ROUTE_COLOR_5 = 'c', ROUTE_COLOR_BOTH = 'y';
static int g_cur_mode;                // 実行中のモード
static std::vector<int> g_route_drawn;  // いま色を付けている区画
static bool g_show_route = true;

static void routeClear(void)
{
  for (int c : g_route_drawn) mmsSend("clearColor %d %d", c / MAZESIZE_MAX, c % MAZESIZE_MAX);
  g_route_drawn.clear();
}
static void routeColorCell(int c, char color)
{
  const int x = c / MAZESIZE_MAX, y = c % MAZESIZE_MAX;
  if (inGoalCell(x, y)) return;
  mmsSend("setColor %d %d %c", x, y, color);
  g_route_drawn.push_back(c);
}
static void routeDraw(const std::vector<int> & route, char color)
{
  routeClear();
  for (int c : route) routeColorCell(c, color);
}

// 経路の列 "NNEE..."(先頭は走り出すときの向き、以降は区画へ入るたびの向き)を、スタート区画から
// たどって通る区画の並びにする
static std::vector<int> routeFromDirs(const char * dirs)
{
  static const char NAME[] = "NESW";
  static const int RX[4] = {0, 1, 0, -1}, RY[4] = {1, 0, -1, 0};
  std::vector<int> route;
  int x = 0, y = 0;
  route.push_back(x * MAZESIZE_MAX + y);
  for (const char * p = dirs; *p; p++) {
    const char * d = strchr(NAME, *p);
    if (!d || !*d) break;
    x += RX[d - NAME];
    y += RY[d - NAME];
    if ((x < 0) || (x >= W) || (y < 0) || (y >= H)) break;
    route.push_back(x * MAZESIZE_MAX + y);
  }
  return route;
}

// fast.cpp が走り出すときに Serial へ出す 2 行を読む
//   "fast path (pivot|slalom): turn N ms, est. M ms" → 推定時間
//   "fast path (len): NNEE..."                       → 経路(区画に色を付ける)
static void serialHook(const char * s)
{
  const char * p = strstr(s, "est. ");
  if (p && strstr(s, "fast path")) g_out.est_ms = atol(p + 5);
  const char * q = strstr(s, "fast path (");
  if (q && isdigit((unsigned char)q[11])) {
    const char * r = strstr(q, "): ");
    if (!r) return;
    std::string dirs(r + 3);
    while (!dirs.empty() && !strchr("NESW", dirs.back())) dirs.pop_back();  // 改行を除く
    g_out.route = routeFromDirs(dirs.c_str());
    if (g_show_route) routeDraw(g_out.route, (g_cur_mode == 4) ? ROUTE_COLOR_4 : ROUTE_COLOR_5);
  }
}

// 全モードの後に、最後に走ったモード 4 と 5 の経路を重ねて塗る
static void routeOverlay(const std::vector<int> & r4, const std::vector<int> & r5)
{
  routeClear();
  for (int c : r4)
    routeColorCell(c, (std::find(r5.begin(), r5.end(), c) != r5.end()) ? ROUTE_COLOR_BOTH : ROUTE_COLOR_4);
  for (int c : r5)
    if (std::find(r4.begin(), r4.end(), c) == r4.end()) routeColorCell(c, ROUTE_COLOR_5);
}

static int cellX(void) { return pose.hx / 2; }
static int cellY(void) { return pose.hy / 2; }

static void simFail(const std::string & why)
{
  char buf[96];
  snprintf(buf, sizeof buf, " @(%d,%d)%s", cellX(), cellY(), DIR_NAME[pose.dir]);
  throw SimAbort{why + buf};
}
static bool atCenter(void)
{
  return (pose.x % SECTION == HALF_SECTION) && (pose.y % SECTION == HALF_SECTION);
}

static int floorDiv(int a, int b) { return (a >= 0) ? a / b : -((-a + b - 1) / b); }
// 位置 p[mm] に最も近い半区画の格子点。ちょうど中間なら進む向き(sg)の側へ寄せる
static int nearestHalf(int p, int sg)
{
  return (sg > 0) ? floorDiv(p + HALF_SECTION / 2, HALF_SECTION)
                  : -floorDiv(-(p - HALF_SECTION / 2), HALF_SECTION);
}

// 真の位置に合わせて mms のマウスを前へ進める。壁に当たると mms が crash を返す
static void mmsSync(void)
{
  const int d = pose.dir;
  const int tx = DX[d] ? nearestHalf(pose.x, DX[d]) : pose.hx;
  const int ty = DY[d] ? nearestHalf(pose.y, DY[d]) : pose.hy;
  const int n = (tx - pose.hx) * DX[d] + (ty - pose.hy) * DY[d];
  if (n < 0) simFail("moved backward");
  if (n == 0) return;
  char cmd[32];
  if (n == 1)
    snprintf(cmd, sizeof cmd, "moveForwardHalf");
  else
    snprintf(cmd, sizeof cmd, "moveForwardHalf %d", n);
  const std::string r = mmsQuery(cmd);
  if (r != "ack") simFail("collision (" + r + ")");
  pose.hx = tx;
  pose.hy = ty;
}

static void forward(int len)
{
  if (len < 0) simFail("negative length");
  // 往路なら、ゴールの区画に入るまでの距離を 1mm 単位で数える
  for (int s = 1; g_out.active && (s <= len); s++) {
    g_out.mm++;
    if (inGoal(pose.x + DX[pose.dir] * s, pose.y + DY[pose.dir] * s)) g_out.active = false;
  }
  pose.x += DX[pose.dir] * len;
  pose.y += DY[pose.dir] * len;
  mmsSync();
}
// slalom=true は slalomCell() の旋回(往路の記録で超信地旋回と分けて数える)
static void turnBy(int k, bool slalom = false)
{
  if (!atCenter()) simFail("turn off-center");
  if (g_out.active) {
    if (slalom)
      g_out.slalom++;
    else
      g_out.pivot += (k < 0) ? -k : k;
  }
  for (int i = 0; i < ((k < 0) ? -k : k); i++) mmsQuery(k > 0 ? "turnRight" : "turnLeft");
  pose.dir = (pose.dir + k + 4) & 3;
}
static void requireWallBehind(void)
{
  if (!mmsBool("wallBack")) simFail("back into no wall");
}

// 区画の中心では今の区画の壁を、区画の境目ではこれから入る区画の壁を見る(tools/step8_sim と同じ)。
// 境目では wallLeft/wallRight が柱を指して常に true になるので、斜め前(wallFrontLeft/Right)で
// 入る区画の左右の壁を、2 半区画先(wallFront 2)で入る区画の前の壁を読む
static void sensorUpdate(void)
{
  const bool center = (pose.hx % 2 == 1) && (pose.hy % 2 == 1);
  const bool f = mmsBool(center ? "wallFront" : "wallFront 2");
  const bool r = mmsBool(center ? "wallRight" : "wallFrontRight");
  const bool l = mmsBool(center ? "wallLeft" : "wallFrontLeft");
  g_sensor.sen_f.is_wall = f;
  g_sensor.sen_f.value = f ? 200 : 0;
  g_sensor.sen_r.is_wall = r;
  g_sensor.sen_l.is_wall = l;
  g_sensor.sen_r.value = r ? g_sensor.sen_r.th_wall_slalom + 50 : 0;
  g_sensor.sen_l.value = l ? g_sensor.sen_l.th_wall_slalom + 50 : 0;
}

// 歩数マップ(MapManager::steps_map)は private なので、実機のコードを変えずに読むため、
// 「明示的インスタンス化ではアクセス制御を確かめない」という C++ の規則でメンバーへのポインタを取り出す
using StepsMember = unsigned short (MapManager::*)[MAZESIZE_MAX][MAZESIZE_MAX];
struct StepsTag
{
  friend StepsMember stepsMember(StepsTag);
};
template <StepsMember M>
struct StepsRob
{
  friend StepsMember stepsMember(StepsTag) { return M; }
};
template struct StepsRob<&MapManager::steps_map>;

// 歩数マップを mms の各区画に文字で出す(前回から変わった所だけ送る)。65535(到達できない)は消す。
// 歩数マップは探索の nextDirGet() を呼んだときに作り直されるので、RUN の呼び出しごとに見れば
// その時点で次の方向を決めるのに使った値になる。最短走行の往路は歩数マップを使わない(ダイクストラ法)
static bool g_show_steps = true;
static long drawn_steps[MAZESIZE_MAX][MAZESIZE_MAX];
static void stepsReset(void)
{
  for (auto & col : drawn_steps)
    for (long & v : col) v = -1;
}
static void drawSteps(void)
{
  if (!g_show_steps) return;
  const auto & steps = g_map.*stepsMember(StepsTag{});
  for (int x = 0; x < W; x++)
    for (int y = 0; y < H; y++) {
      const long v = steps[x][y];
      if (v == drawn_steps[x][y]) continue;
      drawn_steps[x][y] = v;
      if (v == 65535)
        mmsSend("clearText %d %d", x, y);
      else
        mmsSend("setText %d %d %ld", x, y, v);
    }
}

// g_map が知っている壁を mms の画面に描く(前回から変わった所だけ送る)
static char drawn[MAZESIZE_MAX][MAZESIZE_MAX][4];
static void drawReset(void)
{
  memset(drawn, _UNKNOWN, sizeof drawn);
  g_route_drawn.clear();  // 色は clearAllColor で消えるので、記録だけ消す
  stepsReset();
  mmsSend("clearAllColor");
  mmsSend("clearAllText");
  for (int x = 0; x < W; x++)
    for (int y = 0; y < H; y++)
      for (int d = 0; d < 4; d++) mmsSend("clearWall %d %d %c", x, y, DIR_CHAR[d]);
  for (int x = min(GX, GX2); x <= max(GX, GX2); x++)
    for (int y = min(GY, GY2); y <= max(GY, GY2); y++) mmsSend("setColor %d %d G", x, y);
}
static void drawMap(void)
{
  for (int x = 0; x < W; x++)
    for (int y = 0; y < H; y++)
      for (int d = 0; d < 4; d++) {
        const char v = g_map.wallDataGet(x, y, (t_global_direction)d);
        if (v == drawn[x][y][d]) continue;
        drawn[x][y][d] = v;
        if (v == WALL)
          mmsSend("setWall %d %d %c", x, y, DIR_CHAR[d]);
        else
          mmsSend("clearWall %d %d %c", x, y, DIR_CHAR[d]);
      }
}

static void callBegin(const char * name, int arg)
{
  if (mmsBool("wasReset")) throw SimReset{};
  if (++pose.calls > pose.call_limit) simFail("call limit (infinite loop?)");
  drawMap();
  drawSteps();
  if (g_sim_verbose)
    printf(
      "  %-16s %4d  from (%d,%d)%s [%d,%d]mm\n", name, arg, cellX(), cellY(), DIR_NAME[pose.dir],
      pose.x, pose.y);
}

// ---- RUN の置き換え(区画境界・区画中心の幾何だけを再現する。tools/step8_sim と同じ) ----
RUN::RUN()
{
  search_speed = SEARCH_SPEED_INIT;
  max_speed = MAX_SPEED_INIT;
  slalom_speed = SLALOM_SPEED_INIT;
  // 最短走行の経路探索の重み(fast.cppのcostMake())に使う
  search_accel = SEARCH_ACCEL_INIT;
  turn_accel = TURN_ACCEL_INIT;
  tread_width = TREAD_WIDTH_INIT;
  speed = 0;
}
int RUN::stoppableSpeed(int len)
{
  const float v_stop = sqrt(MIN_SPEED * MIN_SPEED + 2.0 * 1000.0 * search_accel * len);
  return (int)min((float)search_speed, v_stop);
}
void RUN::straight(int len, int, int, int finish_speed)
{
  callBegin("straight", len);
  forward(len);
  speed = finish_speed;
  sensorUpdate();
}
void RUN::accelerate(int len, int finish_speed)
{
  callBegin("accelerate", len);
  forward(len);
  speed = finish_speed;
  sensorUpdate();
}
void RUN::oneStep(int len, int)
{
  callBegin("oneStep", len);
  forward(len);
  sensorUpdate();
}
void RUN::decelerate(int len, float)
{
  callBegin("decelerate", len);
  forward(len);
  speed = 0;
  sensorUpdate();
}
// 前壁センサーによる早めの停止は再現せず、常に max_len 進む
bool RUN::oneStepDynamic(int max_len, int, int, bool)
{
  callBegin("oneStepDynamic", max_len);
  forward(max_len);
  sensorUpdate();
  return false;
}
void RUN::rotate(t_local_direction dir, int times)
{
  callBegin(dir == right ? "rotate R" : "rotate L", times);
  turnBy((dir == right ? 1 : -1) * times);
  sensorUpdate();
}
// 区画境界 → 半区画前進 → 90度 → 半区画前進(正味の位置は実機の slalomCell() と同じ)
void RUN::slalomCell(t_local_direction dir)
{
  callBegin(dir == right ? "slalomCell R" : "slalomCell L", 0);
  forward(HALF_SECTION);
  turnBy(dir == right ? 1 : -1, true);
  forward(HALF_SECTION);
  sensorUpdate();
}
// 90度回って後ろの壁に当て、もう一度 90度回って後ろの壁に当てる(実機の uTurn() と同じ順)
void RUN::uTurn(t_local_direction dir)
{
  callBegin(dir == right ? "uTurn R" : "uTurn L", 0);
  const int k = (dir == right) ? 1 : -1;
  turnBy(k);
  requireWallBehind();
  turnBy(k);
  requireWallBehind();
  sensorUpdate();
}
void RUN::startAccelerate(int len, int finish_speed, bool align)
{
  callBegin(align ? "startAcc(align)" : "startAccelerate", len);
  if (align) requireWallBehind();
  forward(len);
  speed = finish_speed;
  sensorUpdate();
}

// ---- 1 モードの実行と判定 ----
static void poseInit(void)
{
  pose.x = pose.y = HALF_SECTION;
  pose.dir = north;
  pose.hx = pose.hy = 1;
  pose.calls = 0;
  pose.call_limit = 40L * W * H + 200;
}

// 1 モード分の結果(最後に比較表として出す)
struct ModeResult
{
  int mode;
  bool ok;
  OutboundLog out;
  bool reached;  // ゴールの区画に入った
};
static std::vector<ModeResult> g_results;

static bool runMode(int mode)
{
  pose.calls = 0;
  sensorUpdate();
  g_out = OutboundLog{true, 0, 0, 0, -1, {}};
  g_cur_mode = mode;
  routeClear();  // 前のモードの経路を消す。最短走行なら走り出すときに自分の経路を塗る
  fprintf(stderr, "mode %d start\n", mode);
  std::string why;
  try {
    g_misc.modeExec(mode);
  } catch (const SimAbort & e) {
    why = e.why;
  }
  drawMap();
  if (why.empty()) {
    char buf[96];
    if (g_map.unreachable)
      why = "unreachable";
    else if (!atCenter() || (cellX() != 0) || (cellY() != 0) || (pose.dir != north)) {
      snprintf(buf, sizeof buf, "not home: (%d,%d)%s", cellX(), cellY(), DIR_NAME[pose.dir]);
      why = buf;
    } else if (
      (g_map.mypos.x != cellX()) || (g_map.mypos.y != cellY()) || (g_map.mypos.dir != pose.dir)) {
      snprintf(
        buf, sizeof buf, "belief (%d,%d)%s != truth", g_map.mypos.x, g_map.mypos.y,
        DIR_NAME[g_map.mypos.dir & 3]);
      why = buf;
    }
  }
  const bool reached = !g_out.active;
  g_out.active = false;
  g_results.push_back(ModeResult{mode, why.empty(), g_out, reached});
  if (!why.empty()) {
    fprintf(stderr, "mode %d FAIL: %s\n", mode, why.c_str());
    return false;
  }
  fprintf(stderr, "mode %d ok (%ld RUN calls)\n", mode, pose.calls);
  return true;
}

// 各モードの往路(スタートからゴールの区画に入るまで)を表にする
static void printSummary(void)
{
  fprintf(stderr, "outbound (start -> goal):\n");
  fprintf(stderr, "  mode  result  cells  turns  pivot  slalom  est.ms\n");
  for (const ModeResult & r : g_results) {
    char est[16] = "-";
    if (r.out.est_ms >= 0) snprintf(est, sizeof est, "%ld", r.out.est_ms);
    if (!r.reached) {
      fprintf(
        stderr, "  %4d  %-6s  %5s  %5s  %5s  %6s  %6s\n", r.mode, r.ok ? "ok" : "FAIL", "-", "-",
        "-", "-", est);
      continue;
    }
    fprintf(
      stderr, "  %4d  %-6s  %5.1f  %5d  %5d  %6d  %6s\n", r.mode, r.ok ? "ok" : "FAIL",
      r.out.mm / (double)SECTION, r.out.pivot + r.out.slalom, r.out.pivot, r.out.slalom, est);
  }
}

static void printStats(void)
{
  static const char * const names[] = {
    "best-run-distance", "best-run-turns", "best-run-effective-distance",
    "total-distance",    "total-turns",    "score"};
  for (const char * n : names) {
    char cmd[64];
    snprintf(cmd, sizeof cmd, "getStat %s", n);
    fprintf(stderr, "  %-28s %s\n", n, mmsQuery(cmd).c_str());
  }
}

static void usage(void)
{
  fprintf(
    stderr,
    "usage: step8_mms [モード ...] [-g x y x2 y2] [-carry] [-nosteps] [-noroute] [-v]\n"
    "  モード  順に実行する modeExec() の番号(既定: 2 4 5)。2/3 は探索、4/5 は最短走行\n"
    "  -g     ゴール区画(既定: 迷路の大きさが parameter.h と同じなら GOAL_*_INIT、違えば中央)\n"
    "  -carry 最短走行の復路で増えた地図を次のモードへ引き継ぐ(既定は直前の探索の地図で比べる)\n"
    "  -nosteps 歩数マップを表示しない\n"
    "  -noroute 最短走行の経路に色を付けない\n"
    "  -v     RUN の呼び出しと Serial の出力をログに出す\n");
}

int main(int argc, char ** argv)
{
  // stdout を mms 専用にし、それ以外の出力は stderr へ回す
  g_mms = fdopen(dup(1), "w");
  dup2(2, 1);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);  // パイプにつながると溜め込まれ、異常終了で失われるため

  std::vector<int> modes;
  int gset = 0, g[4] = {0, 0, 0, 0};
  bool carry = false;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "-v")
      g_sim_verbose = true;
    else if (a == "-nosteps")
      g_show_steps = false;
    else if (a == "-noroute")
      g_show_route = false;
    else if (a == "-carry")
      carry = true;
    else if ((a == "-g") && (i + 4 < argc)) {
      for (int k = 0; k < 4; k++) g[k] = atoi(argv[++i]);
      gset = 1;
    } else if ((a == "-h") || (a == "--help")) {
      usage();
      return 0;
    } else
      modes.push_back(atoi(a.c_str()));
  }
  if (modes.empty()) modes = {2, 4, 5};

  W = atoi(mmsQuery("mazeWidth").c_str());
  H = atoi(mmsQuery("mazeHeight").c_str());
  if ((W < 2) || (W > MAZESIZE_MAX) || (H < 2) || (H > MAZESIZE_MAX)) {
    fprintf(stderr, "maze %dx%d is not supported (2..%d)\n", W, H, MAZESIZE_MAX);
    return 1;
  }
  if (gset) {
    GX = g[0], GY = g[1], GX2 = g[2], GY2 = g[3];
  } else if ((W == MAZESIZE_X_INIT) && (H == MAZESIZE_Y_INIT)) {
    GX = GOAL_X_INIT, GY = GOAL_Y_INIT, GX2 = GOAL_X2_INIT, GY2 = GOAL_Y2_INIT;
  } else {
    // mms がゴールとみなす中央の区画(一辺が偶数なら 2 区画、奇数なら 1 区画)
    GX = (W - 1) / 2, GX2 = W / 2, GY = (H - 1) / 2, GY2 = H / 2;
  }
  g_map.maze_x = W;
  g_map.maze_y = H;
  g_map.goal_mx = GX, g_map.goal_my = GY, g_map.goal_mx2 = GX2, g_map.goal_my2 = GY2;
  g_map.goalValidate();
  if (
    (g_map.goal_mx != GX) || (g_map.goal_my != GY) || (g_map.goal_mx2 != GX2) ||
    (g_map.goal_my2 != GY2)) {
    fprintf(stderr, "goal (%d,%d)-(%d,%d) is invalid for %dx%d\n", GX, GY, GX2, GY2, W, H);
    return 1;
  }
  g_sensor.sen_r.th_wall_slalom = TH_SEN_R_INIT;
  g_sensor.sen_l.th_wall_slalom = TH_SEN_L_INIT;
  fprintf(stderr, "maze %dx%d, goal (%d,%d)-(%d,%d), modes", W, H, GX, GY, GX2, GY2);
  for (int m : modes) fprintf(stderr, " %d", m);
  fprintf(stderr, carry ? ", map carried over\n" : ", fast runs use the last search map\n");
  Serial.hook = serialHook;

  // リセットボタンが押されたら、地図を消して最初のモードからやり直す
  for (;;) {
    try {
      poseInit();
      mapErase();
      drawReset();
      g_results.clear();
      // 最短走行(4/5)は復路の探索で地図が増えるので、既定では直前の探索が終わった時点の地図に
      // 戻してから走らせ、同じ地図で比べる
      static unsigned char search_map[16][16];
      bool have_search_map = false;
      bool ok = true;
      for (int m : modes) {
        if (!ok) break;
        const bool fast = (m == 4) || (m == 5);
        if (fast && !carry && have_search_map) memcpy(flash_map, search_map, sizeof flash_map);
        ok = runMode(m);
        if (!fast) {
          memcpy(search_map, flash_map, sizeof flash_map);
          have_search_map = true;
        }
      }
      fprintf(stderr, ok ? "all modes ok\n" : "stopped\n");
      printSummary();
      // 最後に走ったモード 4 と 5 の経路を重ねて塗る
      {
        const std::vector<int> *r4 = nullptr, *r5 = nullptr;
        for (const ModeResult & r : g_results) {
          if ((r.mode == 4) && !r.out.route.empty()) r4 = &r.out.route;
          if ((r.mode == 5) && !r.out.route.empty()) r5 = &r.out.route;
        }
        if (g_show_route && r4 && r5) {
          routeOverlay(*r4, *r5);
          fprintf(stderr, "route colors: orange = mode 4 only, cyan = mode 5 only, yellow = both\n");
        }
      }
      printStats();
      fprintf(stderr, "press Reset in mms to run again\n");
      while (!mmsBool("wasReset")) usleep(200 * 1000);
      throw SimReset{};
    } catch (const SimReset &) {
      fprintf(stderr, "reset\n");
      mmsQuery("ackReset");
    }
  }
}
