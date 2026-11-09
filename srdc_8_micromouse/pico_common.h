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

// 共通のインクルード。各.cppはこれだけをincludeする。順序に依存するincludeをここにまとめる

#ifndef PICO_COMMON_H_
#define PICO_COMMON_H_

#include <Arduino.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <WiFi.h>

#include "SPIFFS.h"

#include "parameter.h"      // 各種パラメータ定義(マクロのみ)
#include "device.h"         // ピン定義 + デバイス制御関数
#include "map_manager.h"    // 迷路/方向の型定義(t_local_direction 等)
#include "run.h"            // 走行制御(t_local_direction を使用)
#include "sensor.h"         // センサ
#include "misc.h"           // モードメニュー
#include "adjust.h"         // 調整メニュー
#include "fast.h"           // 最短走行
#include "search.h"         // 探索走行
#include "flash.h"          // SPIFFS 読み書き(Flash.cpp)
#include "web.h"            // WebServer(webserver.cpp)

#endif  // PICO_COMMON_H_
