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

#define AP_MODE

AsyncWebServer server(80);

#ifdef AP_MODE
const char * ssid = "SRDC_STD";
const char * password = "12345678";
#else
const char * ssid = "使用しているルータのSSID";
const char * password = "ルータのパスワード";
#endif

// web表示専用の安定化。リクエストごとに.valueを5個貯めてメディアンを返す(STEP8_動作まとめ.md §10)。
// ch: 0=L 1=R 2=F
#define WEB_SEN_MED_N 5

static short webSenStable(unsigned char ch, short v)
{
  static short hist[3][WEB_SEN_MED_N];
  static unsigned char pos[3];
  static bool full[3];

  hist[ch][pos[ch]] = v;
  if (++pos[ch] >= WEB_SEN_MED_N) {
    pos[ch] = 0;
    full[ch] = true;
  }

  unsigned char n = full[ch] ? WEB_SEN_MED_N : pos[ch];
  if (n == 0) return v;  // まだ1個も貯まっていない

  short s[WEB_SEN_MED_N];
  for (unsigned char i = 0; i < n; i++) s[i] = hist[ch][i];
  for (unsigned char i = 1; i < n; i++) {  // 挿入ソート
    short k = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > k) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = k;
  }
  return s[n / 2];  // メディアン
}

// WiFi TX電流バーストによるブラウンアウト対策で送信出力を下げる。詳細はSTEP8_動作まとめ.md §7参照。
#define WEB_WIFI_TX_POWER WIFI_POWER_8_5dBm

// --- トップページ HTML 生成 ----------------------------------------------------
// セクションごとの関数に分割している(STEP8_動作まとめ.md §10参照)

// DOCTYPE〜迷路マップ表示領域(プレースホルダ)まで。値の埋め込みなし。
static String htmlPageHead(void)
{
  return R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>SRDC_STD_WebServer</title>
<style>
  html {
    font-family: Helvetica;
    display: inline-block;
    margin: 0px auto;
    text-align: center;
  }
  h1 {
    font-size: 3.0rem;
    color: blue;
  }
  #map {
    display: inline-block;
    padding: 12px;
    background: #222;
    color: #fff;
    font-family: monospace;
    text-align: left;
    line-height: 1.1;
  }
  .unknown { color: #f44; }
  .virtual { color: #fc0; }
  .notes { text-align: left; max-width: 640px; margin: 0 auto; font-size: 0.85rem; }
  .notes li { margin-bottom: 0.6em; }
</style>
</head>
<body>
<h1>SRDC_STD_Parameter</h1>
<form action="/get">
<p><h2>Maze Map</h2></p>
<pre id="map">Loading...</pre>
<br>
)HTML";
}

// battery Value 表示 (id="voltage" をJSが定期更新)
static String htmlBatterySection(void)
{
  String html = R"HTML(<p><h2>battery Value</h2></p>
<table align="center">
<tr><td id="voltage">)HTML";
  html += String(g_sensor.battery_value);
  html += R"HTML(</td></tr>
</table>
<br>
)HTML";
  return html;
}

// Sensor Value 表示 (id="left_value"等をJSが定期更新)
static String htmlSensorValueSection(void)
{
  String html = R"HTML(<p><h2>Sensor Value</h2></p>
<table align="center" border="1">
<tr><th></th><th>Left</th><th>Right</th></tr>
<tr><th>SIDE VALUE</th><td id="left_value">)HTML";
  html += String(g_sensor.sen_l.value);
  html += R"HTML(</td><td id="right_value">)HTML";
  html += String(g_sensor.sen_r.value);
  html += R"HTML(</td></tr>
<tr><th>FRONT VALUE</th><td colspan="2" id="front_value">)HTML";
  html += String(g_sensor.sen_f.value);
  html += R"HTML(</td></tr>
</table>
<br>
<br>
)HTML";
  return html;
}

// 入力欄1つ。nameはparameters.txtのキーと同じにする(保存時にparamSet()へそのまま渡すため)
static String inputBox(const char * key, const String & value)
{
  return String("<input name=\"") + key + "\" type=\"text\" size=\"10\" value=" + value + ">";
}

// 壁センサ Gain・基準値・しきい値(ベース値)と倍率の入力欄 + Save ボタン
static String htmlSensorParameterSection(void)
{
  const t_sensor_base & b = g_sensor.base;
  String html = R"HTML(<p><h2>Sensor Parameter</h2></p>
<table align="center">
<tr><th>Gain</th><td>)HTML";
  html += inputBox("con_kp", String(g_run.con_wall.kp, 3));
  html += R"HTML(</td></tr>
<tr><th>Sensor ratio</th><td>)HTML";
  html += inputBox("sen_ratio", String(g_sensor.ratio_pct));
  html += R"HTML(%</td></tr>
</table>
<p>下の表はベース値。判定には「ベース値 × Sensor ratio」を使う(括弧内が実際の値)</p>
<table align="center">
<tr><th></th><th>Left</th><th>Right</th></tr>
<tr><th>SIDE REF</th><td>)HTML";
  html += inputBox("ref_sen_l", String(b.ref_l)) + " (" + String(g_sensor.sen_l.ref) + ")";
  html += "</td><td>";
  html += inputBox("ref_sen_r", String(b.ref_r)) + " (" + String(g_sensor.sen_r.ref) + ")";
  html += R"HTML(</td></tr>
<tr><th>SIDE Threshold</th><td>)HTML";
  html += inputBox("th_sen_l", String(b.th_l)) + " (" + String(g_sensor.sen_l.th_wall) + ")";
  html += "</td><td>";
  html += inputBox("th_sen_r", String(b.th_r)) + " (" + String(g_sensor.sen_r.th_wall) + ")";
  html += R"HTML(</td></tr>
<tr><th>FRONT Threshold</th><td colspan="2">)HTML";
  html += inputBox("th_sen_f", String(b.th_f)) + " (" + String(g_sensor.sen_f.th_wall) + ")";
  html += R"HTML(</td></tr>
<tr><th>FRONT measured(wall / no wall)</th><td colspan="2">)HTML";
  html += String(b.f_wall) + " / " + String(b.f_nowall);
  html += R"HTML(</td></tr>
<tr><th>SIDE Threshold(Slalom)</th><td>)HTML";
  html += inputBox("th_sen_l_slalom", String(b.th_l_slalom)) + " (" +
          String(g_sensor.sen_l.th_wall_slalom) + ")";
  html += "</td><td>";
  html += inputBox("th_sen_r_slalom", String(b.th_r_slalom)) + " (" +
          String(g_sensor.sen_r.th_wall_slalom) + ")";
  html += R"HTML(</td></tr>
<tr><th>FRONT Approach Threshold</th><td colspan="2">)HTML";
  html += inputBox("th_sen_f_approach", String(b.th_approach)) + " (" +
          String(g_sensor.th_approach) + ")";
  html += R"HTML(</td></tr>
<tr><th>FRONT Approach Threshold(Slalom)</th><td colspan="2">)HTML";
  html += inputBox("th_sen_f_approach_slalom", String(b.th_approach_slalom)) + " (" +
          String(g_sensor.th_approach_slalom) + ")";
  html += R"HTML(</td></tr>
</table>
<br>
<br>
<input type="submit" value="Save">
<br>
<br>
)HTML";
  return html;
}

// タイヤ直径・トレッドの入力欄
static String htmlTireParameterSection(void)
{
  String html = R"HTML(<p><h2>Tire Parameter</h2></p>
<table align="center">
<tr><th>TIRE_DIAMETER</th><td>)HTML";
  html += inputBox("tire_d", String(g_run.tire_diameter, 3));
  html += R"HTML(mm</td></tr>
<tr><th>TREAD_WIDTH</th><td>)HTML";
  html += inputBox("tread_w", String(g_run.tread_width, 3));
  html += R"HTML(mm</td></tr>
</table>
<br>
<br>
)HTML";
  return html;
}

// 迷路の大きさ・ゴール座標の入力欄
static String htmlGoalParameterSection(void)
{
  String html = R"HTML(<p><h2>GOAL Parameter</h2></p>
<table align="center">
<tr><th></th><th>X</th><th>Y</th></tr>
<tr><th>MAZE SIZE</th><td>)HTML";
  html += inputBox("maze_x", String(g_map.maze_x));
  html += "</td><td>";
  html += inputBox("maze_y", String(g_map.maze_y));
  html += R"HTML(</td></tr>
<tr><th>AXIS</th><td>)HTML";
  html += inputBox("goal_x", String(g_map.goal_mx));
  html += "</td><td>";
  html += inputBox("goal_y", String(g_map.goal_my));
  html += R"HTML(</td></tr>
<tr><th>AXIS2</th><td>)HTML";
  html += inputBox("goal_x2", String(g_map.goal_mx2));
  html += "</td><td>";
  html += inputBox("goal_y2", String(g_map.goal_my2));
  html += R"HTML(</td></tr>
</table>
<p>MAZE SIZEは2〜16。変更して保存すると迷路情報(/map.txt)を消去する</p>
<br>
)HTML";
  return html;
}

// 加減速パラメータ(直進・rotate())の入力欄
static String htmlAccelParameterSection(void)
{
  String html = R"HTML(<p><h2>Accel Parameter</h2></p>
<table align="center">
<tr><th>Search accel</th><td>)HTML";
  html += inputBox("search_accel", String(g_run.search_accel, 3));
  html += R"HTML(</td></tr>
<tr><th>Turn accel</th><td>)HTML";
  html += inputBox("turn_accel", String(g_run.turn_accel, 3));
  html += R"HTML(</td></tr>
</table>
<br>
<br>
)HTML";
  return html;
}

// スラロームの速度・形状の入力欄と、計算した張り出しの表示
static String htmlSlalomParameterSection(void)
{
  String html = R"HTML(<p><h2>Slalom Parameter</h2></p>
<table align="center">
<tr><th>Slalom speed</th><td>)HTML";
  html += inputBox("slalom_speed", String(g_run.slalom_speed));
  html += R"HTML(mm/s</td></tr>
<tr><th>Slalom inner ratio</th><td>)HTML";
  html += inputBox("slalom_inner_ratio", String(g_run.slalom_inner_ratio, 3));
  html += R"HTML(</td></tr>
<tr><th>Slalom outer gain</th><td>)HTML";
  html += inputBox("slalom_outer_gain", String(g_run.slalom_outer_gain, 3));
  html += R"HTML(</td></tr>
<tr><th>Slalom ramp len</th><td>)HTML";
  html += inputBox("slalom_ramp_len", String(g_run.slalom_ramp_len, 1));
  html += R"HTML(mm</td></tr>
<tr><th>Slalom offset(計算値)</th><td>)HTML";
  html += String(g_run.slalom_offset, 1);
  html += R"HTML(mm</td></tr>
</table>
<br>
<br>
)HTML";
  return html;
}

// 速度パラメータ(探索・最短・uTurn)の入力欄
static String htmlSpeedParameterSection(void)
{
  String html = R"HTML(<p><h2>Speed Parameter</h2></p>
<table align="center">
<tr><th>Search speed</th><td>)HTML";
  html += inputBox("search_speed", String(g_run.search_speed));
  html += R"HTML(mm/s</td></tr>
<tr><th>max speed</th><td>)HTML";
  html += inputBox("max_speed", String(g_run.max_speed));
  html += R"HTML(mm/s</td></tr>
<tr><th>Uturn back len</th><td>)HTML";
  html += inputBox("uturn_back_len", String(g_run.uturn_back_len));
  html += R"HTML(mm</td></tr>
<tr><th>Wall return len</th><td>)HTML";
  html += inputBox("wall_return_len", String(g_run.wall_return_len));
  html += R"HTML(mm</td></tr>
<tr><th>Uturn speed</th><td>)HTML";
  html += inputBox("uturn_speed", String(g_run.uturn_speed));
  html += R"HTML(mm/s</td></tr>
</table>
<br>
<br>
)HTML";
  return html;
}

// 速度・加速度プリセット[%]の入力欄 + Save ボタン + フォーム閉じ
static String htmlPresetSection(void)
{
  static const char * const speed_keys[PRESET_NUM] = {
    "preset1_speed", "preset2_speed", "preset3_speed"};
  static const char * const accel_keys[PRESET_NUM] = {
    "preset1_accel", "preset2_accel", "preset3_accel"};
  String html = R"HTML(<p><h2>Preset</h2></p>
<p>走行モードの決定後に選ぶ。上の速度・加速度(Slalom speed含む)に%を掛けて走る</p>
<table align="center">
<tr><th>No</th><th>Speed %</th><th>Accel %</th></tr>
)HTML";
  for (int i = 0; i < PRESET_NUM; i++) {
    html += "<tr><th>" + String(i + 1) + "</th><td>";
    html += inputBox(speed_keys[i], String(g_misc.preset_speed_pct[i]));
    html += "</td><td>";
    html += inputBox(accel_keys[i], String(g_misc.preset_accel_pct[i]));
    html += "</td></tr>\n";
  }
  html += R"HTML(</table>
<br>
<br>
<input type="submit" value="Save">
</form><br>
)HTML";
  return html;
}

// 走行モード・調整モードの番号と挙動の一覧(静的、値の埋め込みなし)。詳細はSTEP8_動作まとめ.md §4・§5参照。
static String htmlModeSection(void)
{
  return R"HTML(<hr>
<div class="notes">
<p><h2>走行モード一覧</h2></p>
<table align="center">
<tr><th>No</th><th>挙動</th></tr>
<tr><td>1</td><td>左手法+壁情報記録。ゴール到達かスイッチ押下で停止しFlashへ保存</td></tr>
<tr><td>2</td><td>足立法探索。前センサで区画境界を判定し、スタート→ゴール→スタートと往復してFlashへ保存</td></tr>
<tr><td>3</td><td>足立法探索(モード2のスラローム版)。区画境界から止まらずに曲がる</td></tr>
<tr><td>4</td><td>最短走行。Flashの迷路情報でゴールへ行き、復路はモード2の探索で戻ってFlashへ保存</td></tr>
<tr><td>5</td><td>最短走行(スラローム版)。復路はモード3の探索で戻ってFlashへ保存</td></tr>
<tr><td>6</td><td>Flash上のマップ情報(/map.txt)をボタン操作のみで消去。モータは動かさない</td></tr>
<tr><td>7</td><td>調整メニューへ移行(モータOFF)</td></tr>
</table>
<p><h2>調整モード一覧(モード7から選択)</h2></p>
<table align="center">
<tr><th>No</th><th>挙動</th></tr>
<tr><td>1</td><td>このWebサーバを起動(AP: 192.168.4.1)</td></tr>
<tr><td>2</td><td>右90°旋回×8(計720°)。旋回前後の向きのズレでTREAD_WIDTHを調整</td></tr>
<tr><td>3</td><td>4x4コース最外周をスラローム旋回でつないで周回。直進→旋回→直進のつなぎ・脱調を確認</td></tr>
<tr><td>4</td><td>前壁が見えるまで直進し区画中心で停止。壁際からの収束具合でGain(姿勢制御Pゲイン)を調整</td></tr>
<tr><td>5</td><td>3区画直進。距離のズレでTIRE_DIAMETERを調整</td></tr>
<tr><td>6</td><td>センサ値・電池電圧をシリアルモニタへ連続表示(いずれかのスイッチで終了)</td></tr>
<tr><td>7</td><td>連続旋回(S字)・旋回直後の直進・U字を一続きに確認</td></tr>
<tr><td>8</td><td>モード2・3の到達判定閾値の実測診断。前壁への走行距離と前センサの値をシリアルへ出力</td></tr>
<tr><td>9</td><td>単発のスラローム旋回の確認(2区画直進した区画で曲がる)。張り出しをシリアルへ出力</td></tr>
<tr><td>10</td><td>前センサ閾値の校正。区画中央に置き、1区画先(次の区画の奥)の前壁あり→なしの順にSW2で測り、平均をFRONT Thresholdへ保存</td></tr>
<tr><td>11〜</td><td>調整メニューを終了し走行モード選択へ戻る</td></tr>
</table>
</div>
)HTML";
}

// 各パラメータの効果の説明文(静的、値の埋め込みなし)
static String htmlParameterNotes(void)
{
  return R"HTML(<hr>
<div class="notes">
<p><h2>各パラメータの効果(目安)</h2></p>
<ul>
<li><b>Gain</b>: 壁センサとの距離ずれを補正する強さ。大きいほど壁の中央へ速く収束するが、大きすぎると蛇行・振動・脱調する。小さいと壁に寄ったままなかなか中央に戻らない。</li>
<li><b>Sensor ratio</b>: センサのベース値(SIDE REF・各Threshold)に掛ける倍率[%]。会場で、練習場と同じ置き方をしたときのセンサ値の比を入れると、しきい値をまとめて合わせられる(例: 会場の値が練習場の0.9倍なら90)。</li>
<li><b>SIDE REF</b>: 区画中央にいるときの横センサの目標値。これがずれていると、Gainを合わせても中央からずれた位置に収束してしまう。</li>
<li><b>SIDE/FRONT Threshold</b>: 壁の有無を判定するしきい値。大きすぎると実際は壁があるのに「壁なし」と誤判定(見落とし)し、小さすぎると壁が無いのに「壁あり」と誤判定して不要に曲がる/止まる。</li>
<li><b>FRONT Approach Threshold</b>: モード2で、走行中に前壁が近づいたら区画境界に着いたとみなすしきい値(前センサの値と比べる)。大きいほど壁の近くまで進む。小さすぎると手前で止まって旋回時に柱へ接触しやすい。</li>
<li><b>FRONT Approach Threshold(Slalom)</b>: モード3用の同しきい値。モード3も区画境界で判定するが、モード2より速く走るため別の値にしてある。大きすぎると旋回が遅れて外側の壁に寄る。</li>
<li><b>SIDE Threshold(Slalom)</b>: モード3用の側壁判定しきい値。大きすぎると壁を見落とし、小さすぎると無い壁を認識して誤ったUターン等になる。</li>
<li><b>TIRE_DIAMETER</b>: 実際より大きい値にすると走行距離を過小評価し、目標距離に届く前に止めるべき所を通り過ぎてオーバーラン気味になる。小さい値だと逆に手前で止まる(アンダーラン)。直進(モード5など)で狙った距離とのズレを見て補正する。</li>
<li><b>TREAD_WIDTH</b>: 実際より大きい値にすると回転量を過小に見積もり、90°回りきる前に旋回を止めてしまう(曲がり不足)。小さい値だと曲がりすぎる(オーバーターン)。720°旋回(モード2)で向きのズレを見て補正する。スラロームの旋回角もこの値で決まる。</li>
<li><b>Search accel</b>: 直進の加減速の急さ。大きいほど短い区間でも最高速に届きやすいが、急すぎるとスリップ・脱調しやすい。小さいと滑らかだが、短い区間では目標速度に届く前に距離を使い切り実効速度が落ちる。</li>
<li><b>Turn accel</b>: <code>rotate()</code>(その場超信地旋回)の加減速の急さ。大きいほど短時間で回れるが、急だと機体が振られる/脱調しやすい。小さいとゆっくり確実だが旋回に時間がかかる。</li>
<li><b>Slalom speed</b>: スラローム旋回中の並進速度(外輪の基準速度)。旋回の軌跡は速度によらないので、変えても曲がる位置は変わらない。速いほど遠心力でスリップしやすい。</li>
<li><b>Slalom inner ratio</b>: 旋回の中央で内輪をSlalom speedの何倍まで落とすか(0〜0.9)。小さいほど小回りになり、張り出し(Slalom offset)が小さくなる。0でほぼ内輪を軸にした旋回。</li>
<li><b>Slalom outer gain</b>: 旋回の中央で外輪をSlalom speedの何倍増しにするか(0〜1.0)。上げると同じ小回りのまま旋回が速く終わるが、外輪が速くなる分スリップしやすい。</li>
<li><b>Slalom ramp len</b>: 旋回の入口・出口で、外輪がこの距離[mm]を走る間に内輪を徐々に落とす/戻す。短いほど張り出しが小さいが、急に曲がり始めるため姿勢が乱れやすい。</li>
<li><b>Slalom offset(計算値)</b>: 上の3つから計算した、区画中心から旋回の開始・終了点までの距離。区画境界から(90 − この値)mm直進してから曲がる。90を超えると区画内に収まらない。</li>
<li><b>Search speed</b>: 迷路探索時の巡航速度。大きいほど探索は速いが、壁センサの読み取りや姿勢制御が追いつかず衝突・誤判定のリスクが増える。モード4の旋回前後とモード2の探索では、半区画で止まり切れる速度(Search accel 1.5で約520mm/s)に自動で抑えられる。</li>
<li><b>max speed</b>: 最短走行(モード4・5)で直線区間を走る最高速度。大きいほどタイムは縮むが、その分加速に必要な距離や、次のカーブ手前での減速が間に合うかの余裕が減る。</li>
<li><b>Uturn back len</b> / <b>Wall return len</b> / <b>Uturn speed</b>: 行き止まりUターン(90度旋回→バック→前進×2)とモード2・3のスタート位置合わせでのバック距離・前進距離と速度。バックは壁に確実に当てるため前進より長くし、壁からの位置は前進距離で決まる。</li>
<li><b>Preset</b>: 走行モードをSW2で決定した後、LED4の点滅中にSW1で選ぶ(LED1〜3の点灯位置が番号)。Search speed・max speed・Slalom speedにSpeed %、Search accel・Turn accelにAccel %を掛けて走る。</li>
</ul>
</div>
)HTML";
}

// フォーム閉じ以降のクライアント側ポーリングスクリプト(静的、値の埋め込みなし)
static String htmlClientScript(void)
{
  return R"HTML(</body>
<script>
var getVoltage = function () {
var xhr = new XMLHttpRequest();
xhr.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
document.getElementById("voltage").innerHTML = this.responseText;
}
};
xhr.open("GET", "/voltage", true);
xhr.send(null);
};
setInterval(getVoltage, 2000);

var getLeftValue = function () {
var xhr = new XMLHttpRequest();
xhr.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
document.getElementById("left_value").innerHTML = this.responseText;
}
};
xhr.open("GET", "/left_value", true);
xhr.send(null);
};
setInterval(getLeftValue, 200);

var getRightValue = function () {
var xhr = new XMLHttpRequest();
xhr.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
document.getElementById("right_value").innerHTML = this.responseText;
}
};
xhr.open("GET", "/right_value", true);
xhr.send(null);
};
setInterval(getRightValue, 200);

var getFrontValue = function () {
var xhr = new XMLHttpRequest();
xhr.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
document.getElementById("front_value").innerHTML = this.responseText;
}
};
xhr.open("GET", "/front_value", true);
xhr.send(null);
};
setInterval(getFrontValue, 200);

var getMap = function () {
var xhr = new XMLHttpRequest();
xhr.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
document.getElementById("map").innerHTML = this.responseText;
}
};
xhr.open("GET", "/map", true);
xhr.send(null);
};
getMap();
setInterval(getMap, 1000);
</script>
</html>
)HTML";
}

// トップページ("/")の全HTMLを組み立てる
static String buildTopPageHtml(void)
{
  String html = htmlPageHead();
  html += htmlBatterySection();
  html += htmlSensorValueSection();
  html += htmlSensorParameterSection();
  html += htmlTireParameterSection();
  html += htmlGoalParameterSection();
  html += htmlAccelParameterSection();
  html += htmlSlalomParameterSection();
  html += htmlSpeedParameterSection();
  html += htmlPresetSection();
  html += htmlModeSection();
  html += htmlParameterNotes();
  html += htmlClientScript();
  return html;
}

// 迷路の壁情報をテキスト図としてHTMLに組み立てる("/map"エンドポイント用)
static String buildMazeMapHtml(void)
{
  String map = "";
  map += "+";
  for (int i = 0; i < g_map.maze_x; i++) {
    char wall = g_map.wallDataGet(i, g_map.maze_y - 1, north);
    if (wall == NOWALL) {
      map += "  +";
    } else if (wall == WALL) {
      map += "--+";
    } else if (wall == _UNKNOWN) {
      map += "<span class=\"unknown\">??</span>+";
    } else {
      map += "<span class=\"virtual\">==</span>+";
    }
  }
  map += "\n";

  for (int j = g_map.maze_y - 1; j >= 0; j--) {
    char west_wall = g_map.wallDataGet(0, j, west);
    if (west_wall == NOWALL) {
      map += " ";
    } else if (west_wall == WALL) {
      map += "|";
    } else if (west_wall == _UNKNOWN) {
      map += "<span class=\"unknown\">?</span>";
    } else {
      map += "<span class=\"virtual\">!</span>";
    }

    for (int i = 0; i < g_map.maze_x; i++) {
      map += "  ";
      char east_wall = g_map.wallDataGet(i, j, east);
      if (east_wall == NOWALL) {
        map += " ";
      } else if (east_wall == WALL) {
        map += "|";
      } else if (east_wall == _UNKNOWN) {
        map += "<span class=\"unknown\">?</span>";
      } else {
        map += "<span class=\"virtual\">!</span>";
      }
    }
    map += "\n+";

    for (int i = 0; i < g_map.maze_x; i++) {
      char south_wall = g_map.wallDataGet(i, j, south);
      if (south_wall == NOWALL) {
        map += "  +";
      } else if (south_wall == WALL) {
        map += "--+";
      } else if (south_wall == _UNKNOWN) {
        map += "<span class=\"unknown\">??</span>+";
      } else {
        map += "<span class=\"virtual\">==</span>+";
      }
    }
    map += "\n";
  }

  return map;
}

// Wi-Fiアクセスポイントとパラメータ設定用Webサーバーを開始
void webServerSetup(void)
{
#ifdef AP_MODE
  WiFi.softAP(ssid, password);
  WiFi.setTxPower(WEB_WIFI_TX_POWER);
  IPAddress myIP = WiFi.softAPIP();
  Serial.println(myIP);
  Serial.printf("WiFi TX power = %d (0.25dBm units)\r\n", WiFi.getTxPower());
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  WiFi.setTxPower(WEB_WIFI_TX_POWER);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
#endif

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", buildTopPageHtml());
  });

  server.on("/voltage", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(g_sensor.battery_value) + "mV");
  });

  server.on("/left_value", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(webSenStable(0, g_sensor.sen_l.value)));
  });

  server.on("/right_value", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(webSenStable(1, g_sensor.sen_r.value)));
  });

  server.on("/front_value", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(webSenStable(2, g_sensor.sen_f.value)));
  });

  server.on("/map", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", buildMazeMapHtml());
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest * request) {
    const short old_maze_x = g_map.maze_x, old_maze_y = g_map.maze_y;
    for (int i = 0; i < paramCount(); i++) {
      const char * key = paramKey(i);
      if (request->hasParam(key)) paramSet(key, request->getParam(key)->value());
    }
    paramCommit(old_maze_x, old_maze_y);
    Serial.println("saved");

    buzzerEnable(INC_FREQ);
    delay(30);
    buzzerDisable();
    request->redirect("/");
  });

  server.begin();
  Serial.println("ESP32_WebServer start!");
}
