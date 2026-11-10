# STEP8 の迷路ロジックを mms で動かす

`srdc_8_micromouse` の迷路ロジックを、実機の `.cpp` を書き換えずに、マイクロマウスシミュレーター [mms](https://github.com/mackorone/mms) の上で動かします。mms の画面で、マウスが探索走行と最短走行をする様子と、`g_map` が覚えた壁を見られます。

対象は `map_manager.cpp`、`search.cpp`、`search_dynamic.cpp`、`fast.cpp`、`misc.cpp` で、`MISC::modeExec()` も実機のものを使います。ランダムな迷路でまとめて確かめるときは [tools/step8_sim](../step8_sim/README.md) を使います。

## 仕組み

- [tools/step8_sim](../step8_sim/README.md) の `sim_shim.h`、`Arduino.h`、`FS.h` をそのまま使います。
- `mms_harness.cpp` で、`RUN` の動作関数を「真の位置（mm）と向きを動かし、mms のマウスを最も近い半区画の格子点へ動かす」関数に置き換えます。動作の幾何は `tools/step8_sim` と同じです。
  - 前進は `moveForwardHalf` で送ります。壁に当たると mms が `crash` を返し、失敗になります。
  - `rotate` は `turnRight`/`turnLeft` を送ります。区画の中心にいなければ失敗です。
  - `slalomCell` は「半区画進む → 90° 回る → 半区画進む」として送ります。
  - `uTurn` とスタート時の壁合わせでは、`wallBack` で後ろに壁があるかを確かめます。
- 壁センサーの値は mms に問い合わせます。区画の中心では今の区画の壁を、区画の境目ではこれから入る区画の壁を見ます。
  - 中心: `wallFront`、`wallRight`、`wallLeft`
  - 境目: `wallFront 2`（入る区画の前の壁）、`wallFrontRight`、`wallFrontLeft`（入る区画の左右の壁）。境目で `wallLeft`/`wallRight` を使うと、柱を指して常に `true` になるためです。
- `g_map` が覚えた壁を、`RUN` の呼び出しのたびに `setWall`/`clearWall` で mms の画面に描きます。ゴールの区画は濃い緑で塗ります。
- 歩数マップ（`MapManager::steps_map`）を、`RUN` の呼び出しのたびに各区画の数字として `setText` で表示します。前回から変わった区画だけを送り、到達できない区画（65535）は空欄にします。
  - 歩数マップは探索の `nextDirGet()` を呼んだときに作り直されます。表示される値はその時点で次に進む方向を決めるのに使った値です。往路はゴールが 0、復路はスタートが 0 になります。探索では未知の壁を「ない」とみなします。
  - 最短走行（モード 4・5）の往路は歩数マップを使わず、走り出す前にダイクストラ法で経路を作ります。そのため往路の間は、直前の探索の値が表示されたままになります。
  - `steps_map` は private なので、ハーネスで C++ の明示的インスタンス化の規則を使って読みます。実機のコードは変えていません。
- stdout は mms とのやり取りに使います。`Serial` とログは stderr に出し、mms のログ欄に表示されます。
- `/map.txt` の代わりに、メモリ上の 16×16 バイトの領域へ `mapWrite`/`mapCopy` します。続けて実行するモードの間では地図を引き継ぎます。

## mms の設定

1. ビルドに使う g++ は `C:\mingw64\bin` にあります。mms から呼べるように、このフォルダを `PATH` に入れておきます。
1. mms のアルゴリズム欄の「+」を押し、次のように設定します。

   | 項目 | 値 |
   | --- | --- |
   | Name | `step8`（任意） |
   | Directory | このリポジトリの直下（`srdc_std`） |
   | Build Command | 下のコマンド |
   | Run Command | `build/step8_mms.exe 2 4` |

   Build Command（1 行で入力します）:

   ```text
   g++ -O2 -std=gnu++17 -w -include tools/step8_sim/sim_shim.h -I tools/step8_sim -I srdc_8_micromouse tools/step8_mms/mms_harness.cpp srdc_8_micromouse/map_manager.cpp srdc_8_micromouse/search.cpp srdc_8_micromouse/search_dynamic.cpp srdc_8_micromouse/fast.cpp srdc_8_micromouse/misc.cpp -o build/step8_mms.exe
   ```

1. 迷路を選び、「Build」→「Run」の順に押します。迷路ファイルは [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles) にあります。

`build` フォルダがなければ先に作っておきます。

## 実行時の引数

```text
build/step8_mms.exe [モード ...] [-g x y x2 y2] [-nosteps] [-v]
```

| 引数 | 内容 |
| --- | --- |
| モード | 順に実行する `modeExec()` の番号。既定は `2 4`（探索してから最短走行）。`3 5` でスラローム版、`2 4 4` のように同じモードを繰り返すこともできる |
| `-g` | ゴール区画。既定は、迷路の大きさが `parameter.h` の初期値（4×4）と同じなら `GOAL_*_INIT`、違えば中央（一辺が偶数なら 2×2 区画、奇数なら 1 区画）。mms の記録は中央に着いたときに付く |
| `-nosteps` | 歩数マップを表示しない。mms とのやり取りが減るので、少し速く走る |
| `-v` | `RUN` の呼び出しと `Serial` の出力をログ欄に出す |

迷路の大きさは mms から読みます。対応するのは一辺が 2〜16 区画の迷路です。

## 判定とリセット

各モードの終わりに、ログ欄へ `mode N ok` か `mode N FAIL: 理由` を出します。判定の条件は `tools/step8_sim` と同じで、次のどれかに当たると失敗です。

| 理由 | 内容 |
| --- | --- |
| `collision` | 壁に当たった（mms が `crash` を返した） |
| `turn off-center` | 区画の中心でない所で回った |
| `back into no wall` | `uTurn()` やスタート時の壁合わせで、当てる先に壁がない |
| `unreachable` | `nextDirGet()` が到達不能と判定した、または `fastPathMake()` が経路を作れなかった |
| `not home` | スタート区画の中心に北向きで戻っていない |
| `belief ... != truth` | `g_map.mypos` が実際の位置と一致しない |
| `call limit` | `RUN` の呼び出しが多すぎる（無限ループの疑い） |

失敗すると、残りのモードは実行しません。全モードが終わると、mms の統計（`getStat`）をログ欄に出して待ちます。

16×16 の迷路で `2 4` を実行したときのログの例です。

```text
maze 16x16, goal (7,7)-(8,8), modes 2 4
mode 2 start
mode 2 ok (469 RUN calls)
mode 4 start
mode 4 ok (214 RUN calls)
all modes ok
  best-run-distance            139
  best-run-turns               25
  best-run-effective-distance  105
  total-distance               760
  total-turns                  182
  score                        220.8
press Reset in mms to run again
```

## mms の統計の読み方

mms のソース（`src/Stats.cpp`、`src/Window.cpp`）で確かめた数え方です。

- 距離の単位は半区画です。`moveForward` も `moveForwardHalf` も半区画の数で数えます。上の例の `best-run-distance 139` は約 69.5 区画です。
- 旋回は 90° を 1 回と数えます。180° の旋回は 2 回です。
- 最良の走行（best-run）は、スタートを出てから中央の区画に着くまでの 1 回分です。
- 実効距離は、1 回の移動コマンドで進む半区画の数を d とすると、d が 3 以上のとき d ÷ 2 + 1、それ以外は d です。リセットすると、次の走行に 15 が足されます。
- `score` は「最良の走行の旋回数 + 最良の走行の実効距離 + 0.1 ×（全体の旋回数 + 全体の実効距離）」で、小さいほど良い値です。

ハーネスは `RUN` の関数 1 回につき移動コマンドを 1 回送るので、実効距離と `score` は、アルゴリズムだけでなく動作の分け方でも変わります。たとえば最短走行の `straight()` は直線をまとめて 1 回で送るので、実効距離が小さくなります。探索の方法を比べるときは `best-run-turns` と `best-run-distance` を見ます。

mms の「Reset」を押すと、マウスをスタートへ戻し、地図を消して最初のモードからやり直します。走行の途中で押しても同じです。

## エディタ

VS Code の clangd は、リポジトリ直下の `.clangd` の設定で、`tools/step8_*` の下のファイルを PC の g++（MinGW）向けとして解析します。スケッチ用の `.vscode/compile_commands.json` は ESP32 向けなので、これらのファイルには使いません。

## 扱わないこと

- `tools/step8_sim` と同じく、壁は常に正しく読める前提です。センサーの雑音、`oneStepDynamic()` の前壁センサーによる早めの停止、壁制御、速度と時間は扱いません。
- mms の画面では、スラロームも「区画の中心で 90° 回る」動きとして表示されます。
- 左手法（モード 1）はスイッチで止める前提なので対象外です。
