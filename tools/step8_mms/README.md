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
- 最短走行（モード 4・5）では走り出す前に作った経路の区画に色を付けます（「最短走行の経路の色」）。
- 歩数マップ（`MapManager::steps_map`）を、`RUN` の呼び出しのたびに各区画の数字として `setText` で表示します。前回から変わった区画だけを送り、到達できない区画（65535）は空欄にします。
  - 歩数マップは探索の `nextDirGet()` を呼んだときに作り直されます。表示される値はその時点で次に進む方向を決めるのに使った値です。往路はゴールが 0、復路はスタートが 0 になります。探索では未知の壁を「ない」とみなします。
  - 最短走行（モード 4・5）の往路は歩数マップを使わず、走り出す前にダイクストラ法で経路を作ります。そのため往路の間は、直前の探索の値が表示されたままになります。
  - `steps_map` は private なので、ハーネスで C++ の明示的インスタンス化の規則を使って読みます。実機のコードは変えていません。
- stdout は mms とのやり取りに使います。`Serial` とログは stderr に出し、mms のログ欄に表示されます。
- `/map.txt` の代わりに、メモリ上の 16×16 バイトの領域へ `mapWrite`/`mapCopy` します。続けて実行するモードの間では地図を引き継ぎます。ただし既定では、最短走行は直前の探索の地図に戻してから走ります（「モード 4 と 5 の比較」）。
- `Serial.printf()` の出力はフック（`Arduino.h` の `SimSerial::hook`）でも受け取り、推定時間を読み取ります。

## mms の設定

1. ビルドに使う g++ は `C:\mingw64\bin` にあります。mms から呼べるように、このフォルダを `PATH` に入れておきます。
1. mms のアルゴリズム欄の「+」を押し、次のように設定します。

   | 項目 | 値 |
   | --- | --- |
   | Name | `step8`（任意） |
   | Directory | このリポジトリの直下（`srdc_std`） |
   | Build Command | 下のコマンド |
   | Run Command | `build/step8_mms.exe 2 4 5` |

   Build Command（1 行で入力します）:

   ```text
   g++ -O2 -std=gnu++17 -w -include tools/step8_sim/sim_shim.h -I tools/step8_sim -I srdc_8_micromouse tools/step8_mms/mms_harness.cpp srdc_8_micromouse/map_manager.cpp srdc_8_micromouse/search.cpp srdc_8_micromouse/search_dynamic.cpp srdc_8_micromouse/fast.cpp srdc_8_micromouse/misc.cpp -o build/step8_mms.exe
   ```

1. 迷路を選び、「Build」→「Run」の順に押します。迷路ファイルは [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles) にあります。

`build` フォルダがなければ先に作っておきます。

## 実行時の引数

```text
build/step8_mms.exe [モード ...] [-g x y x2 y2] [-carry] [-nosteps] [-noroute] [-v]
```

| 引数 | 内容 |
| --- | --- |
| モード | 順に実行する `modeExec()` の番号。既定は `2 4 5`（探索してから、モード 4 と 5 の最短走行を比べる）。`3 5` でスラローム版、`2 4 4` のように同じモードを繰り返すこともできる |
| `-carry` | 最短走行の復路で増えた地図を次のモードへ引き継ぐ。既定では、最短走行は直前の探索の地図から走る（「モード 4 と 5 の比較」） |
| `-g` | ゴール区画。既定は、迷路の大きさが `parameter.h` の初期値（4×4）と同じなら `GOAL_*_INIT`、違えば中央（一辺が偶数なら 2×2 区画、奇数なら 1 区画）。mms の記録は中央に着いたときに付く |
| `-nosteps` | 歩数マップを表示しない。mms とのやり取りが減るので、少し速く走る |
| `-noroute` | 最短走行の経路に色を付けない |
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

失敗すると、残りのモードは実行しません。全モードが終わると（失敗で止まったときも）、各モードの往路の比較表と mms の統計（`getStat`）をログ欄に出して待ちます。

ループのある 16×16 の迷路で、既定の `2 4 5` を実行したときのログの例です（mms の統計の行は省いています）。

```text
maze 16x16, goal (7,7)-(8,8), modes 2 4 5, fast runs use the last search map
mode 2 start
mode 2 ok (113 RUN calls)
mode 4 start
mode 4 ok (124 RUN calls)
mode 5 start
mode 5 ok (55 RUN calls)
all modes ok
outbound (start -> goal):
  mode  result  cells  turns  pivot  slalom  est.ms
     2  ok       27.5     13     13       0       -
     4  ok       27.5     13     13       0   28066
     5  ok       21.5     14      0      14    8983
  best-run-distance            ...
press Reset in mms to run again
```

## モード 4 と 5 の比較

全モードが終わると、各モードの**往路**（スタートを出てから、ゴールの区画に初めて入るまで）を表にして出します。最短走行のモード 4 とモード 5 は、ダイクストラ法の重み（旋回の時間）が違うので、同じ地図でも違う経路を選ぶことがあります（[設計メモ §11](../../srdc_8_micromouse/DESIGN_NOTES.md)）。上の例では、旋回の重いモード 4 が、区画数は多くても旋回の少ない経路を選んでいます。

| 列 | 内容 |
| --- | --- |
| `result` | そのモードの判定（`ok`／`FAIL`） |
| `cells` | ゴールの区画に入るまでに走った距離（区画）。スタート区画の中心から数えるので、0.5 の端数が付く |
| `turns` | 90° 旋回の回数。`pivot`（`rotate()`・`uTurn()` の超信地旋回。180° は 2 回）と `slalom`（`slalomCell()`）の合計 |
| `est.ms` | `fast.cpp` が走り出すときに Serial へ出す推定時間（`fast path ...: est. N ms`）。走行パラメータ（ハーネスでは `parameter.h` の初期値）から計算した値で、探索モードには出ない |

- 既定では、最短走行（モード 4・5）はどれも**直前の探索が終わった時点の地図**から走ります。最短走行の復路は探索なので、そのままだと先に走ったモードの復路で地図が増え、後のモードが有利になるためです。`-carry` を付けると、前のモードの地図をそのまま引き継ぎます（実機と同じ）
- 地図が同じなら、`est.ms` で 2 つのモードの速さを比べられます。mms の画面ではスラロームも区画の中心で回る動きとして表示されるので、走る速さの違いは画面には出ません
- 比べたいモードを並べて指定します。たとえば `3 4 5` はスラローム版の探索（モード 3）の地図で比べます
- 経路の違いは区画の色で見られます（次の節）

## 最短走行の経路の色

最短走行（モード 4・5）が走り出すとき、`fastPathMake()` が作った経路の区画に色を付けます。色はそのモードの往路が終わり、復路の探索に入っても残ります。次のモードが始まると消えます。ゴールの区画は濃い緑のままにします。

| 色 | 区画 |
| --- | --- |
| オレンジ | モード 4 の経路 |
| 水色 | モード 5 の経路 |
| 黄色 | 全モードの後の重ね表示で、モード 4 と 5 の両方が通る区画 |

- 全モードが終わり、モード 4 と 5 の両方を走っていたら、それぞれ最後に走った回の経路を重ねて塗り直します。ログ欄に `route colors: orange = mode 4 only, cyan = mode 5 only, yellow = both` と出ます
- 経路は、`fast.cpp` が走り出すときに Serial へ出す経路の列（`fast path (N): NNEE...`）をフックで読み、スタート区画からたどって求めます。実機のコードは変えていません
- 塗るのは走った跡ではなく、計画した経路です。壁に当たるなど途中で失敗したときも、経路全体が塗られたままになります

## mms の統計の読み方

mms のソース（`src/Stats.cpp`、`src/Window.cpp`）で確かめた数え方です。

- 距離の単位は半区画です。`moveForward` も `moveForwardHalf` も半区画の数で数えます。たとえば `best-run-distance 139` は約 69.5 区画です。
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
