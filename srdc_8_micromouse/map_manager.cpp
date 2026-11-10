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

MapManager g_map;

// 迷路情報とスタート地点、ゴール地点を初期化
MapManager::MapManager()
{
  maze_x = MAZESIZE_X_INIT;
  maze_y = MAZESIZE_Y_INIT;
  mapClear();

  mypos.x = 0;
  mypos.y = 0;
  mypos.dir = north;
  unreachable = false;

  goal_mx = goal_mx2 = 0x07;
  goal_my = goal_my2 = 0x07;
}

// 壁情報を未知の状態(スタート地点周辺と外周のみ既知)へ戻す。現在位置・ゴール座標は変えない。
// 詳細は設計メモ §10参照
void MapManager::mapClear(void)
{
  for (int i = 0; i < maze_x; i++) {
    for (int j = 0; j < maze_y; j++) {
      wall[i][j].north = wall[i][j].east = wall[i][j].south = wall[i][j].west =
        _UNKNOWN;  //迷路情報を未知で初期化する
    }
  }

  for (int i = 0; i < maze_x; i++) {
    wall[i][0].south = WALL;               //四方の壁を追加する(南)
    wall[i][maze_y - 1].north = WALL;  //四方の壁を追加する(北)
  }

  for (int i = 0; i < maze_y; i++) {
    wall[0][i].west = WALL;               //四方の壁を追加する(西)
    wall[maze_x - 1][i].east = WALL;  //四方の壁を追加する(東)
  }
  //   x,y
  wall[0][0].east = wall[1][0].west = WALL;  //スタート地点の右の壁を追加する
  wall[0][0].north = wall[0][1].south = NOWALL;
}

// 現在位置と向きをスタート地点へ戻す
void MapManager::positionInit(void)
{
  mypos.x = mypos.y = 0;
  mypos.dir = north;
}

char MapManager::wallDataGet(unsigned char x, unsigned char y, t_global_direction l_global_dir)
{
  switch (l_global_dir) {
    case north:
      return wall[x][y].north;
      break;
    case west:
      return wall[x][y].west;
      break;
    case south:
      return wall[x][y].south;
      break;
    case east:
      return wall[x][y].east;
      break;
  }
  return 99;
}

void MapManager::wallDataSet(
  unsigned char x, unsigned char y, t_global_direction l_global_dir, char data)
{
  switch (l_global_dir) {
    case north:
      wall[x][y].north = data;
      break;
    case west:
      wall[x][y].west = data;
      break;
    case south:
      wall[x][y].south = data;
      break;
    case east:
      wall[x][y].east = data;
      break;
  }
}

void MapManager::axisUpdate(void)
{
  switch (mypos.dir) {
    case north:
      mypos.y++;
      break;
    case east:
      mypos.x++;
      break;
    case south:
      mypos.y--;
      break;
    case west:
      mypos.x--;
      break;
  }
}

void MapManager::nextDir(t_local_direction l_local_dir)
{
  if (l_local_dir == right) {
    switch (mypos.dir) {
      case north:
        mypos.dir = east;
        break;
      case east:
        mypos.dir = south;
        break;
      case south:
        mypos.dir = west;
        break;
      case west:
        mypos.dir = north;
        break;
    }
  } else if (l_local_dir == left) {
    switch (mypos.dir) {
      case north:
        mypos.dir = west;
        break;
      case east:
        mypos.dir = north;
        break;
      case south:
        mypos.dir = east;
        break;
      case west:
        mypos.dir = south;
        break;
    }
  }
}

// センサの壁判定を現在位置と隣接マスへ記録
void MapManager::wallSet(bool IS_SEN_F, bool IS_SEN_R, bool IS_SEN_L)  //壁情報を記録
{
  switch (mypos.dir) {
    case north:
      wall[mypos.x][mypos.y].north = IS_SEN_F ? WALL : NOWALL;
      wall[mypos.x][mypos.y].east = IS_SEN_R ? WALL : NOWALL;
      wall[mypos.x][mypos.y].west = IS_SEN_L ? WALL : NOWALL;
      if (mypos.y < (maze_y - 1)) wall[mypos.x][mypos.y + 1].south = IS_SEN_F ? WALL : NOWALL;
      if (mypos.x < (maze_x - 1)) wall[mypos.x + 1][mypos.y].west = IS_SEN_R ? WALL : NOWALL;
      if (mypos.x > 0) wall[mypos.x - 1][mypos.y].east = IS_SEN_L ? WALL : NOWALL;
      break;
    case east:
      wall[mypos.x][mypos.y].east = IS_SEN_F ? WALL : NOWALL;
      wall[mypos.x][mypos.y].south = IS_SEN_R ? WALL : NOWALL;
      wall[mypos.x][mypos.y].north = IS_SEN_L ? WALL : NOWALL;
      if (mypos.x < (maze_x - 1)) wall[mypos.x + 1][mypos.y].west = IS_SEN_F ? WALL : NOWALL;
      if (mypos.y > 0) wall[mypos.x][mypos.y - 1].north = IS_SEN_R ? WALL : NOWALL;
      if (mypos.y < (maze_y - 1)) wall[mypos.x][mypos.y + 1].south = IS_SEN_L ? WALL : NOWALL;
      break;
    case south:
      wall[mypos.x][mypos.y].south = IS_SEN_F ? WALL : NOWALL;
      wall[mypos.x][mypos.y].west = IS_SEN_R ? WALL : NOWALL;
      wall[mypos.x][mypos.y].east = IS_SEN_L ? WALL : NOWALL;
      if (mypos.y > 0) wall[mypos.x][mypos.y - 1].north = IS_SEN_F ? WALL : NOWALL;
      if (mypos.x > 0) wall[mypos.x - 1][mypos.y].east = IS_SEN_R ? WALL : NOWALL;
      if (mypos.x < (maze_x - 1)) wall[mypos.x + 1][mypos.y].west = IS_SEN_L ? WALL : NOWALL;
      break;
    case west:
      wall[mypos.x][mypos.y].west = IS_SEN_F ? WALL : NOWALL;
      wall[mypos.x][mypos.y].north = IS_SEN_R ? WALL : NOWALL;
      wall[mypos.x][mypos.y].south = IS_SEN_L ? WALL : NOWALL;
      if (mypos.x > 0) wall[mypos.x - 1][mypos.y].east = IS_SEN_F ? WALL : NOWALL;
      if (mypos.y < (maze_y - 1)) wall[mypos.x][mypos.y + 1].south = IS_SEN_R ? WALL : NOWALL;
      if (mypos.y > 0) wall[mypos.x][mypos.y - 1].north = IS_SEN_L ? WALL : NOWALL;
      break;
  }
}

// 最短歩数が同じ方向の優先度を比較して次の方向を選択
void MapManager::stepMapSet(
  unsigned char posX, unsigned char posY, t_global_direction l_global_dir, int * little,
  t_global_direction * now_dir, int * priority)
{
  int tmp_priority;
  tmp_priority = priorityGet(posX, posY, l_global_dir);
  if (steps_map[posX][posY] < *little) {
    *little = steps_map[posX][posY];
    *now_dir = l_global_dir;
    *priority = tmp_priority;
  } else if (steps_map[posX][posY] == *little) {
    if (*priority < tmp_priority) {
      *now_dir = l_global_dir;
      *priority = tmp_priority;
    }
  }
}

t_local_direction MapManager::nextGdir(t_global_direction * p_global_dir)
{
  switch (*p_global_dir) {
    case north:
      switch (mypos.dir) {
        case north:
          return front;
          break;
        case east:
          return left;
          break;
        case south:
          return rear;
          break;
        case west:
          return right;
          break;
        default:
          return loca_dir_error;
          break;
      }
      break;
    case east:
      switch (mypos.dir) {
        case east:
          return front;
          break;
        case south:
          return left;
          break;
        case west:
          return rear;
          break;
        case north:
          return right;
          break;
        default:
          return loca_dir_error;
          break;
      }
      break;
    case south:
      switch (mypos.dir) {
        case south:
          return front;
          break;
        case west:
          return left;
          break;
        case north:
          return rear;
          break;
        case east:
          return right;
          break;
        default:
          return loca_dir_error;
          break;
      }
      break;
    case west:
      switch (mypos.dir) {
        case west:
          return front;
          break;
        case north:
          return left;
          break;
        case east:
          return rear;
          break;
        case south:
          return right;
          break;
        default:
          return loca_dir_error;
          break;
      }
      break;
    default:
      return loca_dir_error;
      break;
  }
}

t_local_direction MapManager::nextDirGet(
  char x, char y, char x2, char y2, t_global_direction * p_global_dir)
{
  int little, priority;

  searchMapMake(x, y, x2, y2);
  little = 65535;
  priority = 0;

  if ((wall[mypos.x][mypos.y].north != WALL) && (mypos.y < (maze_y - 1))) {
    stepMapSet(mypos.x, mypos.y + 1, north, &little, p_global_dir, &priority);
  }
  if ((wall[mypos.x][mypos.y].east != WALL) && (mypos.x < (maze_x - 1))) {
    stepMapSet(mypos.x + 1, mypos.y, east, &little, p_global_dir, &priority);
  }
  if ((wall[mypos.x][mypos.y].south != WALL) && (mypos.y > 0)) {
    stepMapSet(mypos.x, mypos.y - 1, south, &little, p_global_dir, &priority);
  }
  if ((wall[mypos.x][mypos.y].west != WALL) && (mypos.x > 0)) {
    stepMapSet(mypos.x - 1, mypos.y, west, &little, p_global_dir, &priority);
  }

  if (steps_map[mypos.x][mypos.y] == 65535) {
    unreachableWait();
    return loca_dir_error;
  } else {
    return nextGdir(p_global_dir);
  }

  return front;
}

// 現在位置が(x,y)-(x2,y2)の長方形(対角2座標、順不同)に入っているか
bool MapManager::posIn(short x, short y, short x2, short y2)
{
  const short px = mypos.x, py = mypos.y;
  return (px >= min(x, x2)) && (px <= max(x, x2)) && (py >= min(y, y2)) && (py <= max(y, y2));
}

// 迷路の大きさが2〜MAZESIZE_MAXの範囲外なら初期値へ戻す(設計メモ §9)
void MapManager::mazeSizeValidate(void)
{
  if ((maze_x < 2) || (maze_x > MAZESIZE_MAX) || (maze_y < 2) || (maze_y > MAZESIZE_MAX)) {
    Serial.printf("maze size %dx%d invalid -> fallback to init\r\n", maze_x, maze_y);
    maze_x = MAZESIZE_X_INIT;
    maze_y = MAZESIZE_Y_INIT;
  }
}

// ゴール座標が迷路外、またはスタート(0,0)を含む長方形なら初期値へ戻す(設計メモ §9)
void MapManager::goalValidate(void)
{
  const short v[4] = {goal_mx, goal_my, goal_mx2, goal_my2};
  const short lim[4] = {maze_x, maze_y, maze_x, maze_y};
  bool ok = true;
  for (int k = 0; k < 4; k++) {
    if ((v[k] < 0) || (v[k] >= lim[k])) ok = false;
  }
  // 座標が0以上なら、スタート(0,0)を含むのはx・yとも最小値が0のとき
  if (ok && (min(goal_mx, goal_mx2) == 0) && (min(goal_my, goal_my2) == 0)) ok = false;
  if (!ok) {
    Serial.printf(
      "goal (%d,%d)-(%d,%d) invalid -> fallback to init\r\n", goal_mx, goal_my, goal_mx2,
      goal_my2);
    goal_mx = GOAL_X_INIT;
    goal_my = GOAL_Y_INIT;
    goal_mx2 = GOAL_X2_INIT;
    goal_my2 = GOAL_Y2_INIT;
    // 初期値も迷路の外なら、スタートと反対の角の1区画にする(設計メモ §9)
    if ((max(goal_mx, goal_mx2) >= maze_x) || (max(goal_my, goal_my2) >= maze_y)) {
      goal_mx = goal_mx2 = maze_x - 1;
      goal_my = goal_my2 = maze_y - 1;
    }
  }
}

// 歩数マップを未到達で埋め、目標の長方形内の全区画を0にする(複数区画ゴール、設計メモ §9)
void MapManager::stepMapInit(int x, int y, int x2, int y2)
{
  for (int i = 0; i < maze_x; i++) {
    for (int j = 0; j < maze_y; j++) {
      steps_map[i][j] = 65535;
    }
  }
  for (int i = max(min(x, x2), 0); i <= min(max(x, x2), maze_x - 1); i++) {
    for (int j = max(min(y, y2), 0); j <= min(max(y, y2), maze_y - 1); j++) {
      steps_map[i][j] = 0;
    }
  }
}

void MapManager::searchMapMake(int x, int y, int x2, int y2)
{
  bool change_flag;

  stepMapInit(x, y, x2, y2);

  do {
    change_flag = false;
    for (int i = 0; i < maze_x; i++) {
      for (int j = 0; j < maze_y; j++) {
        if (steps_map[i][j] == 65535) continue;
        if (
          (j < (maze_y - 1)) && (wall[i][j].north != WALL) && (steps_map[i][j + 1] == 65535)) {
          steps_map[i][j + 1] = steps_map[i][j] + 1;
          change_flag = true;
        }

        if ((i < (maze_x - 1)) && (wall[i][j].east != WALL) && (steps_map[i + 1][j] == 65535)) {
          steps_map[i + 1][j] = steps_map[i][j] + 1;
          change_flag = true;
        }

        if ((j > 0) && (wall[i][j].south != WALL) && (steps_map[i][j - 1] == 65535)) {
          steps_map[i][j - 1] = steps_map[i][j] + 1;
          change_flag = true;
        }

        if ((i > 0) && (wall[i][j].west != WALL) && (steps_map[i - 1][j] == 65535)) {
          steps_map[i - 1][j] = steps_map[i][j] + 1;
          change_flag = true;
        }
      }
    }
  } while (change_flag == true);
}

int MapManager::priorityGet(unsigned char x, unsigned char y, t_global_direction dir)
{
  int priority;

  priority = 0;

  if (mypos.dir == dir) {
    priority = 2;
  } else if (
    ((mypos.dir == north) && (dir == south)) || ((mypos.dir == east) && (dir == west)) ||
    ((mypos.dir == south) && (dir == north)) || ((mypos.dir == west) && (dir == east))) {
    priority = 0;
  } else {
    priority = 1;
  }

  if (
    (wall[x][y].north == _UNKNOWN) || (wall[x][y].east == _UNKNOWN) ||
    (wall[x][y].south == _UNKNOWN) || (wall[x][y].west == _UNKNOWN)) {
    priority += 4;
  }

  return priority;
}

// ゴールへ行けないことを点滅で知らせる。接触不良による誤判定もあるため、モータを止めて
// スイッチで抜けられるようにする。抜けた後は止めずにすぐ返す(設計メモ §8)
void MapManager::unreachableWait(void)
{
  while (!unreachable && (switchGet() == 0)) {
    motorDisable();
    ledSet(0x0a);
    delay(500);
    ledSet(0x05);
    delay(500);
  }
  unreachable = true;
}

// ---- 最短走行の経路探索(ダイクストラ法、設計メモ §11) ----
// 状態は「区画(x,y)へ向きdirで区画境界から入ったところ」。番号は (x*MAZESIZE_MAX + y)*4 + dir
// 状態から状態へは「k区画直進 → 左右どちらかへ90度旋回」の1動作で移る。旋回した先の区画へ入った状態が次の状態

#define PATH_NODE_MAX (MAZESIZE_MAX * MAZESIZE_MAX * 4)
#define PATH_NO_PREV 0xFFFF

static const signed char dir_dx[4] = {0, 1, 0, -1};  // north, east, south, west
static const signed char dir_dy[4] = {1, 0, -1, 0};

static uint32_t path_dist[PATH_NODE_MAX];  // 始点からの時間[us]
static uint16_t path_prev[PATH_NODE_MAX];  // 直前の状態。始点はPATH_NO_PREV
static bool path_done[PATH_NODE_MAX];      // 時間が確定したか

static inline int pathNode(int x, int y, int dir) { return (x * MAZESIZE_MAX + y) * 4 + dir; }

bool MapManager::fastPathMake(
  short x, short y, short x2, short y2, const t_path_cost & cost, uint32_t * total_us)
{
  const short gx_min = min(x, x2), gx_max = max(x, x2);
  const short gy_min = min(y, y2), gy_max = max(y, y2);
  auto inGoal = [&](int cx, int cy) {
    return (cx >= gx_min) && (cx <= gx_max) && (cy >= gy_min) && (cy <= gy_max);
  };
  auto inMaze = [&](int cx, int cy) {
    return (cx >= 0) && (cx < maze_x) && (cy >= 0) && (cy < maze_y);
  };
  auto passable = [&](int cx, int cy, int dir) {
    return wallDataGet(cx, cy, (t_global_direction)dir) == NOWALL;
  };

  fast_path_len = fast_path_idx = 0;
  *total_us = 0;
  if (inGoal(mypos.x, mypos.y)) return true;  // すでにゴールの中にいる

  for (int i = 0; i < PATH_NODE_MAX; i++) {
    path_dist[i] = UINT32_MAX;
    path_prev[i] = PATH_NO_PREV;
    path_done[i] = false;
  }

  // 始点：区画の中心で静止した状態から、向きを合わせて隣の区画へ入る
  for (int d = 0; d < 4; d++) {
    const int nx = mypos.x + dir_dx[d], ny = mypos.y + dir_dy[d];
    if (!inMaze(nx, ny) || !passable(mypos.x, mypos.y, d)) continue;
    const int times = (d - (int)mypos.dir + 4) % 4;  // 右回りに何回90度回るか
    path_dist[pathNode(nx, ny, d)] = cost.rotate[(times == 3) ? 1 : times];
  }

  uint32_t best = UINT32_MAX;
  int best_node = -1, best_k = 0;
  const int node_num = maze_x * MAZESIZE_MAX * 4;  // x<maze_xの範囲だけ見ればよい

  // 優先度付きキューは使わず、毎回すべての状態から時間が最小のものを探す(状態は最大1024個)
  while (1) {
    int u = -1;
    uint32_t du = UINT32_MAX;
    for (int i = 0; i < node_num; i++) {
      if (!path_done[i] && (path_dist[i] < du)) {
        du = path_dist[i];
        u = i;
      }
    }
    if ((u < 0) || (du >= best)) break;  // 残りの状態からは今の最良より速くならない
    path_done[u] = true;

    const int ux = (u / 4) / MAZESIZE_MAX, uy = (u / 4) % MAZESIZE_MAX, h = u % 4;
    if (inGoal(ux, uy)) {  // 入った直後にゴール。止まるだけ
      if (du + cost.goal[0] < best) {
        best = du + cost.goal[0];
        best_node = u;
        best_k = 0;
      }
      continue;
    }

    // k区画直進した先の区画(ex,ey)で旋回する。直進の途中でゴールに入ったらそこで止まる
    int ex = ux, ey = uy;
    for (int k = 0; k < MAZESIZE_MAX; k++) {
      if (k > 0) {
        if (!passable(ex, ey, h)) break;
        ex += dir_dx[h];
        ey += dir_dy[h];
        if (!inMaze(ex, ey)) break;
        if (inGoal(ex, ey)) {
          if (du + cost.goal[k] < best) {
            best = du + cost.goal[k];
            best_node = u;
            best_k = k;
          }
          break;
        }
      }
      for (int t = 1; t <= 3; t += 2) {  // 右(h+1)と左(h+3)
        const int nh = (h + t) % 4;
        const int nx = ex + dir_dx[nh], ny = ey + dir_dy[nh];
        if (!inMaze(nx, ny) || !passable(ex, ey, nh)) continue;
        const int v = pathNode(nx, ny, nh);
        const uint32_t dv = du + cost.straight[k] + cost.turn;
        if (!path_done[v] && (dv < path_dist[v])) {
          path_dist[v] = dv;
          path_prev[v] = (uint16_t)u;
        }
      }
    }
  }

  if (best_node < 0) return false;
  *total_us = best;

  // 終点から始点へたどり、区画ごとの方向を逆順に積む
  int len = 0;
  const int bh = best_node % 4;
  for (int i = 0; i < best_k; i++) fast_path[len++] = bh;  // 最後の直進
  int v = best_node;
  while (path_prev[v] != PATH_NO_PREV) {
    const int u = path_prev[v];
    const int ux = (u / 4) / MAZESIZE_MAX, uy = (u / 4) % MAZESIZE_MAX, h = u % 4;
    const int vx = (v / 4) / MAZESIZE_MAX, vy = (v / 4) % MAZESIZE_MAX, nh = v % 4;
    // 旋回した区画(vの1つ手前)までの直進区画数
    const int ex = vx - dir_dx[nh], ey = vy - dir_dy[nh];
    const int k = abs(ex - ux) + abs(ey - uy);
    if (len + k + 2 > (int)sizeof(fast_path)) return false;  // 通常は起きない
    fast_path[len++] = nh;  // 旋回した区画では新しい向きで出る
    for (int i = 0; i < k; i++) fast_path[len++] = h;
    v = u;
  }
  fast_path[len++] = v % 4;  // 走行開始時の向き
  for (int i = 0, j = len - 1; i < j; i++, j--) {
    const unsigned char tmp = fast_path[i];
    fast_path[i] = fast_path[j];
    fast_path[j] = tmp;
  }
  fast_path_len = len;
  return true;
}

t_local_direction MapManager::fastPathNext(t_global_direction * p_global_dir)
{
  if (fast_path_idx >= fast_path_len) {
    // 経路の列が尽きた。経路の作り方に誤りがなければ起きない
    Serial.printf("fast path exhausted at (%d,%d)\r\n", mypos.x, mypos.y);
    unreachableWait();
    return loca_dir_error;
  }
  *p_global_dir = (t_global_direction)fast_path[fast_path_idx++];
  return nextGdir(p_global_dir);
}

void MapManager::fastPathPrint(void)
{
  static const char name[4] = {'N', 'E', 'S', 'W'};
  static char buf[sizeof(fast_path) + 1];
  for (int i = 0; i < fast_path_len; i++) buf[i] = name[fast_path[i]];
  buf[fast_path_len] = '\0';
  Serial.printf("fast path (%d): %s\r\n", fast_path_len, buf);
}
