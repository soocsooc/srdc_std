// STEP8 の迷路ロジック(map_manager/search/search_dynamic/fast/misc の .cpp)を実機のまま PC で動かし、
// ランダムな迷路で探索走行と最短走行を確かめる。RUN の動作は「真の位置と向きを動かすだけ」に置き換え、
// 壁センサーの値は正解の迷路と真の位置から作る。使い方は README.md 参照
#include <cstdlib>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "sim_shim.h"  // ビルドでは -include でも読むが、エディタの解析用に明示する

unsigned long g_sim_ms;
bool g_sim_verbose;
SimSerial Serial;

RUN g_run;
SENSOR g_sensor;
ADJUST g_adjust;

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

// ---- 正解の迷路と真の姿勢 ----
static const int DX[4] = {0, 1, 0, -1}, DY[4] = {1, 0, -1, 0};  // north, east, south, west
static const char * const DIR_NAME[4] = {"N", "E", "S", "W"};
static int W, H, GX, GY, GX2, GY2;
static unsigned char truth[MAZESIZE_MAX][MAZESIZE_MAX];  // bit d: d 方向に壁

static bool isWall(int x, int y, int d)
{
  if ((x < 0) || (x >= W) || (y < 0) || (y >= H)) return true;
  return (truth[x][y] >> d) & 1;
}
static void setWall(int x, int y, int d)
{
  truth[x][y] |= 1 << d;
  const int nx = x + DX[d], ny = y + DY[d];
  if ((nx >= 0) && (nx < W) && (ny >= 0) && (ny < H)) truth[nx][ny] |= 1 << ((d + 2) & 3);
}
static void clearWall(int x, int y, int d)
{
  truth[x][y] &= ~(1 << d);
  const int nx = x + DX[d], ny = y + DY[d];
  if ((nx >= 0) && (nx < W) && (ny >= 0) && (ny < H)) truth[nx][ny] &= ~(1 << ((d + 2) & 3));
}
static bool inGoal(int x, int y)
{
  return (x >= min(GX, GX2)) && (x <= max(GX, GX2)) && (y >= min(GY, GY2)) && (y <= max(GY, GY2));
}

struct SimAbort
{
  std::string why;
};

// 真の姿勢。位置は mm で、区画 (cx,cy) の中心が (cx*SECTION+HALF_SECTION, ...)
static struct
{
  int x, y, dir, cx, cy;
  long calls, cells;  // RUN の呼び出し回数、区画に入った回数
  long goal_cells;    // 最初にゴール区画へ入るまでの区画数(-1: 未到達)
  long call_limit;
} pose;

static void simFail(const std::string & why)
{
  char buf[96];
  snprintf(buf, sizeof buf, " @(%d,%d)%s", pose.cx, pose.cy, DIR_NAME[pose.dir]);
  throw SimAbort{why + buf};
}
static bool atCenter(void)
{
  return (pose.x % SECTION == HALF_SECTION) && (pose.y % SECTION == HALF_SECTION);
}

// 1mm ずつ進め、区画の境目を越えるときに壁がないかを確かめる
static void forward(int len)
{
  if (len < 0) simFail("negative length");
  const int d = pose.dir;
  for (int i = 0; i < len; i++) {
    const int nx = pose.x + DX[d], ny = pose.y + DY[d];
    const int mcx = (int)floor((pose.x + nx) * 0.5 / SECTION);
    const int mcy = (int)floor((pose.y + ny) * 0.5 / SECTION);
    if ((mcx != pose.cx) || (mcy != pose.cy)) {
      if (isWall(pose.cx, pose.cy, d)) simFail("collision");
      pose.cx = mcx;
      pose.cy = mcy;
      ++pose.cells;
      if ((pose.goal_cells < 0) && inGoal(pose.cx, pose.cy)) pose.goal_cells = pose.cells;
    }
    pose.x = nx;
    pose.y = ny;
  }
}
static void turnBy(int k)
{
  if (!atCenter()) simFail("turn off-center");
  pose.dir = (pose.dir + k + 4) & 3;
}
static void requireWallBehind(void)
{
  if (!isWall(pose.cx, pose.cy, (pose.dir + 2) & 3)) simFail("back into no wall");
}

// センサーは真の位置の少し先(LOOKAHEAD)にある区画の壁を見る。境目付近では入る区画、中心では今の区画
static const int LOOKAHEAD = HALF_SECTION / 2;
static void sensorUpdate(void)
{
  const int d = pose.dir;
  const int cx = (int)floor((double)(pose.x + DX[d] * LOOKAHEAD) / SECTION);
  const int cy = (int)floor((double)(pose.y + DY[d] * LOOKAHEAD) / SECTION);
  const bool f = isWall(cx, cy, d), r = isWall(cx, cy, (d + 1) & 3),
             l = isWall(cx, cy, (d + 3) & 3);
  g_sensor.sen_f.is_wall = f;
  g_sensor.sen_f.value = f ? 200 : 0;
  g_sensor.sen_r.is_wall = r;
  g_sensor.sen_l.is_wall = l;
  g_sensor.sen_r.value = r ? g_sensor.sen_r.th_wall_slalom + 50 : 0;
  g_sensor.sen_l.value = l ? g_sensor.sen_l.th_wall_slalom + 50 : 0;
}

static void callBegin(const char * name, int arg)
{
  if (++pose.calls > pose.call_limit) simFail("call limit (infinite loop?)");
  if (g_sim_verbose)
    printf(
      "  %-16s %4d  from (%d,%d)%s [%d,%d]mm\n", name, arg, pose.cx, pose.cy, DIR_NAME[pose.dir],
      pose.x, pose.y);
}

// ---- RUN の置き換え(区画境界・区画中心の幾何だけを再現する) ----
RUN::RUN()
{
  search_speed = SEARCH_SPEED_INIT;
  max_speed = MAX_SPEED_INIT;
  slalom_speed = SLALOM_SPEED_INIT;
  speed = 0;
}
int RUN::stoppableSpeed(int) { return search_speed; }
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
  turnBy(dir == right ? 1 : -1);
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

// ---- 迷路の生成と最短距離 ----
static int bfsDist(void)
{
  std::vector<int> dist(W * H, -1), q;
  dist[0] = 0;
  q.push_back(0);
  for (size_t h = 0; h < q.size(); h++) {
    const int x = q[h] % W, y = q[h] / W;
    if (inGoal(x, y)) return dist[q[h]];
    for (int d = 0; d < 4; d++) {
      if (isWall(x, y, d)) continue;
      const int n = (x + DX[d]) + (y + DY[d]) * W;
      if (dist[n] >= 0) continue;
      dist[n] = dist[q[h]] + 1;
      q.push_back(n);
    }
  }
  return -1;
}

// 内側の壁を密度 p で置き、外周・スタートの東壁を足し、ゴール区画の内側の壁を除く。ゴールへ行けなければ作り直す
static int makeMaze(std::mt19937 & rng, double p)
{
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (;;) {
    memset(truth, 0, sizeof truth);
    for (int x = 0; x < W; x++)
      for (int y = 0; y < H; y++)
        for (int d = 0; d < 4; d++) {
          const int nx = x + DX[d], ny = y + DY[d];
          if ((nx < 0) || (nx >= W) || (ny < 0) || (ny >= H)) truth[x][y] |= 1 << d;  // 外周
        }
    for (int x = 0; x < W; x++)
      for (int y = 0; y < H; y++) {
        if ((x + 1 < W) && (u(rng) < p)) setWall(x, y, east);
        if ((y + 1 < H) && (u(rng) < p)) setWall(x, y, north);
      }
    setWall(0, 0, east);
    clearWall(0, 0, north);
    for (int x = min(GX, GX2); x <= max(GX, GX2); x++)
      for (int y = min(GY, GY2); y <= max(GY, GY2); y++) {
        if (x < max(GX, GX2)) clearWall(x, y, east);
        if (y < max(GY, GY2)) clearWall(x, y, north);
      }
    const int best = bfsDist();
    if (best > 0) return best;
  }
}

static void printMaze(void)
{
  for (int y = H - 1; y >= 0; y--) {
    for (int x = 0; x < W; x++) printf("+%s", isWall(x, y, north) ? "---" : "   ");
    printf("+\n");
    for (int x = 0; x < W; x++)
      printf(
        "%c %c ", isWall(x, y, west) ? '|' : ' ',
        (x == 0 && y == 0) ? 'S'
        : inGoal(x, y)     ? 'G'
                           : ' ');
    printf("|\n");
  }
  for (int x = 0; x < W; x++) printf("+---");
  printf("+\n");
}

// ---- 1 モードの実行と判定 ----
struct Stat
{
  long runs = 0, ok = 0, optimal = 0, cells = 0;
  std::map<std::string, long> fails;
  std::string first_fail;
};

static bool runMode(int mode, int best, int maze_index, Stat & st)
{
  pose.x = pose.y = HALF_SECTION;
  pose.dir = north;
  pose.cx = pose.cy = 0;
  pose.calls = pose.cells = 0;
  pose.goal_cells = -1;
  pose.call_limit = 40L * W * H + 200;
  sensorUpdate();
  if (g_sim_verbose) printf("mode %d\n", mode);

  ++st.runs;
  std::string why;
  try {
    g_misc.modeExec(mode);
  } catch (const SimAbort & e) {
    why = e.why;
  }
  if (why.empty()) {
    char buf[96];
    if (g_map.unreachable)
      why = "unreachable";
    else if (pose.goal_cells < 0)
      why = "goal not reached";
    else if (!atCenter() || (pose.cx != 0) || (pose.cy != 0) || (pose.dir != north)) {
      snprintf(buf, sizeof buf, "not home: (%d,%d)%s", pose.cx, pose.cy, DIR_NAME[pose.dir]);
      why = buf;
    } else if (
      (g_map.mypos.x != pose.cx) || (g_map.mypos.y != pose.cy) || (g_map.mypos.dir != pose.dir)) {
      snprintf(
        buf, sizeof buf, "belief (%d,%d)%s != truth", g_map.mypos.x, g_map.mypos.y,
        DIR_NAME[g_map.mypos.dir & 3]);
      why = buf;
    }
  }
  if (!why.empty()) {
    const std::string key = why.substr(0, why.find(" @"));
    ++st.fails[key.substr(0, key.find(':'))];
    if (st.first_fail.empty()) st.first_fail = "maze " + std::to_string(maze_index) + ": " + why;
    if (g_sim_verbose) printf("  => FAIL: %s\n", why.c_str());
    return false;
  }
  ++st.ok;
  st.cells += pose.cells;
  if (pose.goal_cells == best) ++st.optimal;
  if (g_sim_verbose)
    printf("  => ok (%ld cells, goal after %ld, best %d)\n", pose.cells, pose.goal_cells, best);
  return true;
}

static void usage(void)
{
  printf(
    "usage: step8_sim [迷路数=1000] [壁密度=0.35] [迷路の一辺=parameter.hの値]\n"
    "                 [-g x y x2 y2] [-seed n] [-i 番号] [-v]\n"
    "  -g    ゴール区画(既定: 迷路の一辺が parameter.h と同じなら GOAL_*_INIT、違えば右上の角)\n"
    "  -i    その番号の迷路だけを実行(-v と合わせて失敗の再現に使う)\n"
    "  -v    迷路と RUN の呼び出しを表示\n");
}

int main(int argc, char ** argv)
{
  long n = 1000, only = -1;
  unsigned seed = 1;
  double p = 0.35;
  int size = MAZESIZE_X_INIT, gset = 0, g[4] = {0, 0, 0, 0}, pos = 0;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "-v")
      g_sim_verbose = true;
    else if ((a == "-seed") && (i + 1 < argc))
      seed = atoi(argv[++i]);
    else if ((a == "-i") && (i + 1 < argc))
      only = atol(argv[++i]);
    else if ((a == "-g") && (i + 4 < argc)) {
      for (int k = 0; k < 4; k++) g[k] = atoi(argv[++i]);
      gset = 1;
    } else if ((a == "-h") || (a == "--help")) {
      usage();
      return 0;
    } else if (pos == 0 && ++pos)
      n = atol(a.c_str());
    else if (pos == 1 && ++pos)
      p = atof(a.c_str());
    else if (pos == 2 && ++pos)
      size = atoi(a.c_str());
  }
  if (g_sim_verbose) setvbuf(stdout, NULL, _IONBF, 0);
  if ((size < 2) || (size > MAZESIZE_MAX)) {
    usage();
    return 1;
  }
  W = H = size;
  if (gset) {
    GX = g[0], GY = g[1], GX2 = g[2], GY2 = g[3];
  } else if (size == MAZESIZE_X_INIT) {
    GX = GOAL_X_INIT, GY = GOAL_Y_INIT, GX2 = GOAL_X2_INIT, GY2 = GOAL_Y2_INIT;
  } else {
    GX = GX2 = GY = GY2 = size - 1;
  }
  g_map.maze_x = W;
  g_map.maze_y = H;
  g_map.goal_mx = GX, g_map.goal_my = GY, g_map.goal_mx2 = GX2, g_map.goal_my2 = GY2;
  g_map.goalValidate();
  if (
    (g_map.goal_mx != GX) || (g_map.goal_my != GY) || (g_map.goal_mx2 != GX2) ||
    (g_map.goal_my2 != GY2)) {
    printf("goal (%d,%d)-(%d,%d) is invalid for %dx%d\n", GX, GY, GX2, GY2, W, H);
    return 1;
  }
  g_sensor.sen_r.th_wall_slalom = TH_SEN_R_INIT;
  g_sensor.sen_l.th_wall_slalom = TH_SEN_L_INIT;

  // 探索モードのあと、その地図で最短モードを走らせる(最短モードの帰りの探索で地図が変わるので毎回戻す)
  const int search_modes[] = {2, 3}, fast_modes[] = {4, 5};
  std::map<int, Stat> stats;
  unsigned char searched_map[16][16];
  for (long i = 0; i < n; i++) {
    if ((only >= 0) && (i != only)) continue;
    std::mt19937 rng(seed * 1000003u + (unsigned)i);
    const int best = makeMaze(rng, p);
    if (g_sim_verbose) {
      printf("=== maze %ld (shortest %d)\n", i, best);
      printMaze();
    }
    for (int sm : search_modes) {
      mapErase();
      if (!runMode(sm, best, i, stats[sm]))
        continue;  // 地図が保存されていないので最短モードは飛ばす
      memcpy(searched_map, flash_map, sizeof flash_map);
      for (int fm : fast_modes) {
        memcpy(flash_map, searched_map, sizeof flash_map);
        runMode(fm, best, i, stats[sm * 10 + fm]);
      }
    }
  }

  printf(
    "%dx%d goal (%d,%d)-(%d,%d), wall density %.2f, seed %u, %ld mazes\n", W, H, GX, GY, GX2, GY2,
    p, seed, (only >= 0) ? 1 : n);
  printf("%-14s %6s %6s %8s %10s  failures\n", "mode", "runs", "ok", "optimal", "avg cells");
  for (auto & kv : stats) {
    const Stat & s = kv.second;
    char name[32];
    if (kv.first < 10)
      snprintf(name, sizeof name, "%d (search)", kv.first);
    else
      snprintf(name, sizeof name, "%d after %d", kv.first % 10, kv.first / 10);
    printf(
      "%-14s %6ld %6ld %8ld %10.1f ", name, s.runs, s.ok, s.optimal,
      s.ok ? (double)s.cells / s.ok : 0.0);
    for (auto & f : s.fails) printf(" %s=%ld", f.first.c_str(), f.second);
    printf("\n");
    if (!s.first_fail.empty()) printf("%14s first: %s\n", "", s.first_fail.c_str());
  }
  return 0;
}
