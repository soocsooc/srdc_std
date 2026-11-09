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

#ifndef SRC_SEARCH_H_
#define SRC_SEARCH_H_

#include "map_manager.h"  // t_local_direction, t_global_direction

class SEARCH
{
public:
  void lefthandGetWall(char gx, char gy, char gx2, char gy2);
  // 足立法。前進にはoneStepDynamic()を使い、前壁が見えたら区画境界とみなす(設計メモ §4)
  void adachiDynamic(char gx, char gy, char gx2, char gy2);
  // adachiDynamic()の旋回をslalomCell()にした版(設計メモ §3)
  void adachiSlalomDynamic(char gx, char gy, char gx2, char gy2);
  // 区画中心で180度回転する。前壁と左右どちらかの壁があればuTurn()、なければrotate()(設計メモ §5)
  void turnBack(bool wall_f, bool wall_r, bool wall_l);
};

extern SEARCH g_search;

#endif /* SRC_SEARCH_H_ */