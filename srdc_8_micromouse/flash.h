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

#ifndef SRC_FLASH_H_
#define SRC_FLASH_H_

#include <Arduino.h>
#include <FS.h>

void flashInit(void);
void flashBegin(void);
void mapWrite(void);
void mapCopy(void);
void mapErase(void);
void paramWrite(void);
void paramRead(void);
void paramApply(void);
void paramCommit(short old_maze_x, short old_maze_y);
void paramPrint(void);
void paramSerialPoll(void);
bool paramSet(const String & key, const String & value);
int paramCount(void);
const char * paramKey(int index);
int split(String data, char delimiter, String * dst);
void writeFile(fs::FS & fs, String path, String message);
void appendFile(fs::FS & fs, String path, String message);

#endif /* SRC_FLASH_H_ */
