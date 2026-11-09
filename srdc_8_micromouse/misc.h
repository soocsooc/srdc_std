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

#ifndef SRC_MISC_H_
#define SRC_MISC_H_

// 走行モードは1〜5、MODE_ADJUSTで調整メニュー。「次」で1つ進め、MODE_ADJUSTの次は1
#define MODE_ERASE_MAP 6
#define MODE_ADJUST 7

class MISC
{
public:
  unsigned char mode_select;
  void modeExec(int mode);
  void modeListPrint(void);  // 走行モード一覧をシリアルへ出力
  short buttonInc(short _data, short limit, short limit_data);
  void buttonOk(void);
  void goalAppeal(void);
};

extern MISC g_misc;

#endif /* SRC_MISC_H_ */
