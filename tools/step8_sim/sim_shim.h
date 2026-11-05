// g++ の -include で各 .cpp の先頭に読ませ、実機の pico_common.h の代わりにする。
// PICO_COMMON_H_ を先に定義して本物を無効にし、Wi-Fi・SPIFFS 以外のヘッダは実機のものを使う
#ifndef SIM_SHIM_H_
#define SIM_SHIM_H_
#define PICO_COMMON_H_

#include <Arduino.h>  // tools/step8_sim/Arduino.h

// 順序に依存があるので pico_common.h と同じ順に並べる
// clang-format off
#include "parameter.h"
#include "device.h"
#include "map_manager.h"
#include "run.h"
#include "sensor.h"
#include "misc.h"
#include "adjust.h"
#include "fast.h"
#include "search.h"
#include "flash.h"
// clang-format on

#endif  // SIM_SHIM_H_
