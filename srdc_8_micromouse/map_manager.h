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

#ifndef MAP_MANAGER_H_
#define MAP_MANAGER_H_

#define MAZESIZE_MAX (16)  //配列の大きさ。実際の迷路の大きさはmaze_x/maze_y(設計メモ §9)

#define _UNKNOWN 2  //壁があるかないか判らない状態の場合の値
#define NOWALL 0    //壁がない場合の値
#define WALL 1      //壁がある場合の値

typedef struct
{
  unsigned char west : 2;   //西の壁情報bit7-6
  unsigned char south : 2;  //南の壁情報 bit5-4
  unsigned char east : 2;   //東の壁情報 bit3-2
  unsigned char north : 2;  //北の壁情報 bit1-0
} t_wall;                   //壁情報を格納する構造体(ビットフィールド)

typedef enum { front, right, left, rear, loca_dir_error } t_local_direction;

typedef enum { north, east, south, west, glob_dir_error } t_global_direction;

typedef struct
{
  unsigned char x;
  unsigned char y;
  t_global_direction dir;
} t_position;

// 最短走行の経路探索(ダイクストラ法)の重み。各動作の所要時間[us](設計メモ §11)
typedef struct
{
  uint32_t straight[MAZESIZE_MAX];  // k区画直進して旋回へつなぐ時間。k=0は0
  uint32_t goal[MAZESIZE_MAX];  // k区画直進してゴール区画の中心で止まるまでの時間
  uint32_t turn;                // 90度旋回1回(区画境界から次の区画境界まで)
  uint32_t rotate[3];           // 走行開始時の向き合わせ(0/90/180度)
} t_path_cost;

class MapManager
{
public:
  MapManager();

  t_position mypos;
  short maze_x, maze_y;  // 迷路の大きさ[区画](Web UIで変更可、設計メモ §9)
  // ゴール区画(長方形の対角2座標)。1区画のときは2座標を同じにする(設計メモ §9)
  short goal_mx, goal_my, goal_mx2, goal_my2;
  // ゴールへ行けないと判定し、スイッチで抜けたときtrue。modeExec()が復路以降を中断するのに使う(設計メモ §8)
  bool unreachable;

  void axisUpdate(void);
  void nextDir(t_local_direction dir);
  // 目標は(x,y)-(x2,y2)の長方形。スタートへ戻るときは(0,0,0,0)
  t_local_direction nextDirGet(char x, char y, char x2, char y2, t_global_direction * dir);
  // 壁がないと確定した辺(NOWALL)だけを通り、costの合計時間が最小になる経路を現在位置から作る。
  // 到達できなければfalse。total_usに推定時間を返す(設計メモ §11)
  bool fastPathMake(
    short x, short y, short x2, short y2, const t_path_cost & cost, uint32_t * total_us);
  // fastPathMake()で作った経路の次の方向。区画へ入るたびに呼ぶ(nextDirGet()と同じ戻り値)
  t_local_direction fastPathNext(t_global_direction * dir);
  void fastPathPrint(void);  // 経路をSerialへ出す
  // ゴールへ行けないことを点滅で知らせ、スイッチで抜けるまで待つ(設計メモ §8)
  void unreachableWait(void);
  bool posIn(short x, short y, short x2, short y2);  // 現在位置が長方形内か
  void mazeSizeValidate(void);  // 迷路の大きさが範囲外なら初期値へ戻す
  void goalValidate(void);  // ゴール座標が迷路外・スタートを含むなら初期値へ戻す
  void positionInit(void);
  void mapClear(void);
  void wallDataSet(unsigned char x, unsigned char y, t_global_direction dir, char data);
  char wallDataGet(unsigned char x, unsigned char y, t_global_direction dir);
  void wallSet(bool IS_SEN_F, bool IS_SEN_R, bool IS_SEN_L);

private:
  unsigned short steps_map[MAZESIZE_MAX][MAZESIZE_MAX];  //歩数マップ
  t_wall wall[MAZESIZE_MAX][MAZESIZE_MAX];  //壁の情報を格納する構造体配列
  // fastPathMake()の経路。区画へ入るたびに進む向き(東西南北)の列で、先頭は走行開始時の向き。
  // 同じ区画を別の向きで2回通ることもあるので、長さの上限は状態の数(区画数×4)+1
  unsigned char fast_path[MAZESIZE_MAX * MAZESIZE_MAX * 4 + 1];
  short fast_path_len, fast_path_idx;

  void stepMapSet(
    unsigned char posX, unsigned char posY, t_global_direction l_global_dir, int * little,
    t_global_direction * now_dir, int * priority);
  t_local_direction nextGdir(t_global_direction * p_global_dir);
  void searchMapMake(int x, int y, int x2, int y2);
  void stepMapInit(int x, int y, int x2, int y2);
  int priorityGet(unsigned char x, unsigned char y, t_global_direction dir);
};

extern MapManager g_map;

#endif  // MAP_MANAGER_H_