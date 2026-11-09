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

// LEDはHIGHで点灯する。LED0〜3は回路図のL_1〜L_4
#define LED0 5
#define LED1 13
#define LED2 16
#define LED3 15

// SW1=次のモード、SW2=決定。SW2(GPIO35)は内部プルアップがないので、SW2をプルアップして
// いないならUSE_SW2を0にする(SW2を読まず、SW1の短押し=次、長押し=決定になる)
#define SW1 33
#define SW2 35
#define USE_SW2 1
#define LONG_PRESS_MS 1000

#define BUZZER 23

#define INC_FREQ 2000
#define DEC_FREQ 1000

#define FREQ_C 523  //ド
#define FREQ_D 587  //レ
#define FREQ_E 659  //ミ

#define SW_NEXT 1
#define SW_OK 2

char g_mode;

void ledSet(char data);
void modeExec(char mode);
char switchGet(void);

void setup()
{
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT);

  ledcAttach(BUZZER, 440, 10);  // Pin, 周波数[Hz](440~4000), PWM分解能[bit](2^bit-1, 最大値: 14)
  ledcWrite(BUZZER, 1024);      // Pin, pwm_duty(0~1023(2^10-1), LOW(0) or HIGH(1024) -> Mute)

  g_mode = 1;
  ledSet(g_mode);
}

void loop()
{
  char sw = switchGet();

  // 「次」でmodeを1つ進め(15の次は1)、押したときに音を鳴らす
  if (sw == SW_NEXT) {
    g_mode++;
    if (g_mode > 15) {
      g_mode = 1;
    }
    ledcWriteTone(BUZZER, INC_FREQ);  // Pin, 周波数[Hz]
    delay(30);
    ledcWrite(BUZZER, 1024);
    ledSet(g_mode);
  }

  // 「決定」で効果音->消音->mode音
  if (sw == SW_OK) {
    ledcWriteTone(BUZZER, INC_FREQ);
    delay(80);
    ledcWriteTone(BUZZER, DEC_FREQ);
    delay(80);
    ledcWrite(BUZZER, 1024);
    delay(300);
    modeExec(g_mode);
  }
}

// スイッチが押されて離されるまで待ち、SW_NEXTかSW_OKを返す
char switchGet(void)
{
  while (1) {
    if (USE_SW2 && (digitalRead(SW2) == LOW)) {
      delay(30);  // チャタリング回避
      while (digitalRead(SW2) == LOW) {
        continue;
      }
      delay(30);
      return SW_OK;
    }
    if (digitalRead(SW1) == LOW) {
      unsigned long t0 = millis();
      delay(30);  // チャタリング回避
      while (digitalRead(SW1) == LOW) {
        continue;
      }
      unsigned long len = millis() - t0;
      delay(30);
      return (!USE_SW2 && (len >= LONG_PRESS_MS)) ? SW_OK : SW_NEXT;
    }
  }
}

// data(2進数4bit)の各bitに応じてLEDを点灯(HIGH)・消灯(LOW)する
void ledSet(char data)
{
  digitalWrite(LED0, (data & 0x01) ? HIGH : LOW);
  digitalWrite(LED1, (data & 0x02) ? HIGH : LOW);
  digitalWrite(LED2, (data & 0x04) ? HIGH : LOW);
  digitalWrite(LED3, (data & 0x08) ? HIGH : LOW);
}

// modeに応じた高さの音を1秒再生
void modeExec(char mode)
{
  switch (mode) {
    case 1:
      ledcWriteTone(BUZZER, FREQ_C);  // ド
      delay(1000);
      ledcWrite(BUZZER, 1024);
      break;
    case 2:
      ledcWriteTone(BUZZER, FREQ_D);  // レ
      delay(1000);
      ledcWrite(BUZZER, 1024);
      break;
    case 3:
      ledcWriteTone(BUZZER, FREQ_E);  // ミ
      delay(1000);
      ledcWrite(BUZZER, 1024);
      break;
    default:
      ledcWrite(BUZZER, 1024);  // 消音
      break;
  }
}
