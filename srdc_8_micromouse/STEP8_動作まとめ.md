# STEP8「迷路を走破する」動作まとめ

対象: `pc4_8_micromouse`
出典: `PiCoClassic4_arduino_software_manual_1.0.pdf` p.71〜 ＋ 実プロジェクトのソース
（`C:\Users\groov\Documents\Programs\Arduino\Mouse\pc4\pc4_8_micromouse`）

---

## 1. STEP8 の位置づけ

本番会場での走行用に、走行モード3・7・8・9・10と調整メニューだけを残した`pc4_9_competition`を切り出した（2026-09-25）。変更はSTEP8で行い、完成したものを手作業でSTEP9へ移す。本番会場での調整手順は同ディレクトリの`STEP9_本番調整手順.md`、切り出しの内容は[スラローム機能_実装まとめ.md §2.55](スラローム機能_実装まとめ.md)参照。

- STEP1〜7 は各ハードウェア（LED / スイッチ / ブザー / センサ / モータ / 壁センサ P 制御）のデバイスドライバ確認用サンプル。
- **STEP8 はそれらのドライバを使って「迷路を探索走行して走破する」マイクロマウス競技用アプリケーション**。
- 流用性を考えて機能ごとに複数ファイルへ分割されている。

### ファイル構成（実プロジェクト）

| ファイル | 役割 |
| --- | --- |
| `pc4_8_micromouse.ino` | `setup()` / `loop()`。初期化とモード選択 |
| `device.cpp` / `device.h` | GPIO・タイマ割り込み・ブザー・LED・スイッチ・センサ AD・電池電圧などハード依存処理とピン定義 |
| `run.cpp` / `run.h` | 走行制御（速度割り込み、姿勢 P 制御、加減速・旋回関数、スラローム旋回） |
| `sensor.cpp` / `sensor.h` | センサ計測割り込み、壁判定、姿勢制御用誤差、電池 2 色 LED 表示 |
| `search.cpp` / `search.h` | 左手法 `lefthand()` と足立法 `adachi()`。スラローム版 `lefthandSlalom()`/`adachiSlalom()` を追加（本フォークで追加、`adachiSlalom()`の旋回は`slalomCell()`） |
| `fast.cpp` / `fast.h` | 既知迷路情報による最短走行 `run()` |
| `map_manager.cpp` / `map_manager.h` | 壁情報管理・歩数マップ生成・次進行方向決定 |
| `Flash.cpp` / `flash.h` | SPIFFS への迷路情報・パラメータ読み書き |
| `misc.cpp` / `misc.h` | スイッチ操作、ゴールアピール、走行モード実行 `modeExec()` |
| `adjust.cpp` / `adjust.h` | 調整メニュー（センサ確認・直径・トレッド・スラローム確認）。`mapView()` はメニューから外れて呼び出しがなくなったため削除（§2.57） |
| `webserver.cpp` / `web.h` | Wi-Fi 経由でパラメータを確認・変更する Web サーバ |
| `parameter.h` | タイヤ径・トレッド・センサ基準値・しきい値・速度などの初期値 |
| `TMC5240.cpp` / `.h` | ステッピングモータドライバ制御（16 マイクロステップ）。`readXactualLR()` で左右パルスを符号付き個別取得可能（スラロームの並進・回転分離用、本フォークで追加） |

`pico_common.h` が全 `.cpp` の共通インクルード（`.ino` 連結をやめて個別 `.cpp` としてビルドする構成）。

---

## 2. 起動シーケンス（`setup()`）

`pc4_8_micromouse.ino:17`

1. `deviceInit()`：GPIO、タイマ0、センサ計測タスク、シリアル(115200) を初期化
   - タイマ0：`timerAlarm(g_timer0, 1000, ...)` → **1 kHz、走行速度制御 `controlInterrupt()`**（ISR、core1）
   - **`sensorTask`**：`xTaskCreatePinnedToCore(..., core 1, prio 20)` → 1 ms 周期で `sensorInterrupt()` を 5 回（電池電圧処理1回＋前後左右 1 巡、各センサ実効 1 kHz）。旧構成のタイマ1（4 kHz ISR）を置き換え。WiFi/AsyncTCP（core0）と分離するため（→ §7「WiFi 接続中の…」）
2. `flashBegin()`：SPIFFS マウント。**中央（黄）スイッチを押しながら起動すると `flashInit()`** で `parameter.h` の初期値を書き戻す
3. `paramRead()`：全パラメータを初期値にしてから SPIFFS の `/parameters.txt` を読み、センサのベース値・倍率、タイヤ径・トレッド・ゲイン・速度・スラローム形状・プリセット・迷路の大きさ・ゴール座標を各モジュールへ反映。最後に`paramApply()`で `g_run.pulse`（`tire_diameter*π/(200*16)`）、センサの判定値（ベース値×倍率、§9）、スラロームの張り出し`slalom_offset`（§6）を計算する
   - 続けて `mapCopy()` で `/map.txt` の迷路情報を読み込む（本フォークで追加。電源を切っても前回の地図から探索を続けられ、Web UI の迷路図も残る。[スラローム機能_実装まとめ.md §2.61](スラローム機能_実装まとめ.md)）
4. `g_tmc5240.init()`：モータドライバ設定
5. 起動音（`buzzerEnable(INC_FREQ)` 80 ms）→ `motorDisable()`
6. `g_misc.mode_select = 1`（モード選択画面から開始）

---

## 3. モード選択（`loop()`）

`pc4_8_micromouse.ino:35`

- 選択中モード番号を LED に **2 進数表示**（`ledSet(mode_select)`）
- 右スイッチ `SW_RM` → `buttonInc`：番号 +1（15 を超えたら 1 に戻す、`INC_FREQ` 3000 Hz を 30 ms）
- 左スイッチ `SW_LM` → `buttonDec`：番号 -1（1 未満で 15 に戻す、`DEC_FREQ` 2000 Hz）
- 中央スイッチ `SW_CM` → `buttonOk()`（確認音）+ `g_misc.modeRun(mode_select)` を実行
  - 走行モード（1〜9）では先に**プリセット選択**に入る（本フォークで追加）。LED4 が点滅し、LED1〜3 の点灯位置がプリセット番号（1〜3）。左右スイッチで変更、中央で決定。前回選んだ番号から始まる
  - 選んだプリセットの速度%を`search_speed`・`max_speed`・`slalom_speed`に、加速度%を`search_accel`・`turn_accel`に掛けて`modeExec()`を実行し、終わったらベース値へ戻す（[スラローム機能_実装まとめ.md §2.64](スラローム機能_実装まとめ.md)）
- `switchGet()` はチャタリング対策に 20 ms 待ってスイッチが離れるまで待機。複数同時押しは値を加算（青4＋赤1＝5）
- 本フォークで追加：`loop()`の先頭で`paramSerialPoll()`を呼び、待機中は USB シリアルからのパラメータ設定コマンドを受け付ける（§9）

---

## 4. 走行モード一覧（`misc.cpp` `MISC::modeExec`）

各モード実行時、先頭で実行モードを`mode_select`へ記録して`motorEnable()` + `delay(1000)`、末尾で`ledSet(mode)`により実行したモード番号のLED表示へ戻して`motorDisable()`（全モード共通、詳細は[スラローム機能_実装まとめ.md §2.44](スラローム機能_実装まとめ.md)）。

| 番号 | 関数 | 動作 |
| --- | --- | --- |
| **1** | `g_search.lefthand()` | 左手法。座標・壁情報を管理しないため**ゴールでも止まらない**。**いずれかのスイッチを押すと次の区画境界で減速停止し、モード選択へ戻る**（本フォークで追加） |
| **2** | 足立法探索 | スタート→ゴール探索→ゴールアピール→スタートへ帰還→迷路情報を Flash 保存 |
| **3** | 最短走行 | Flash の迷路情報を読み出し、直進区間をまとめて高速走行でゴールへ。**復路はモード8の探索（`adachiDynamic()`）で戻り**、Flash へ保存（[スラローム機能_実装まとめ.md §2.62](スラローム機能_実装まとめ.md)） |
| **4** | 迷路情報を取得する左手法 | 左手法で走りながら通過区画の壁を `g_map` に記録。ゴール到達 or スイッチ押下で停止し `mapWrite()` で Flash 保存（本フォークで追加） |
| **5** | 左手法（スラローム版） | `lefthand()` のスラローム版。90°旋回を減速・停止せず`slalom()`で曲がる（本フォークで追加） |
| **6** | 足立法（スラローム版） | `adachi()` のスラローム版。90°旋回は`slalomCell()`。往復してゴールアピール・Flash 保存する構成はモード2と同じ（本フォークで追加） |
| **7** | 最短走行（スラローム版） | `FAST::run()` のスラローム版 `FAST::runSlalom()`。90°旋回を`slalomCell()`で停止せず曲がる。**復路はモード9の探索（`adachiSlalomDynamic()`）で戻り**、Flash へ保存（本フォークで追加、[スラローム機能_実装まとめ.md §2.62](スラローム機能_実装まとめ.md)） |
| **8** | 足立法（センサ閾値主導の動的到達判定版） | `adachi()`の`front`分岐のみ`oneStepDynamic()`に置き換えた`adachiDynamic()`。往復・ゴールアピール・Flash 保存の構成はモード2・6と同じ。開始時の`startAccelerate()`でスタート位置を固定（§2.69）。デバッグ用に認識した壁情報をLEDへ表示する(モード9と同じ、§2.36)（本フォークで追加。到達判定は2026-09-25にFR+FLの和`th_approach_sum`(290)へ変更、和判定での実車確認は未実施。詳細は[スラローム機能_実装まとめ.md §2.18〜§2.24・§2.46](スラローム機能_実装まとめ.md)） |
| **9** | 足立法（センサ閾値主導の動的到達判定版・スラローム版） | `adachiDynamic()`と同じ骨組みで、90°旋回を区画境界からの`slalomCell()`にした`adachiSlalomDynamic()`。前進は`oneStepDynamic()`で区画境界まで（しきい値はモード9専用の`th_approach_sum_slalom`）、側壁はモード9専用の`th_wall_slalom`で判定、後ろへ戻るときは`turnBack()`（当てる壁があれば`uTurn()`、なければ`rotate()`）。認識した壁情報をLEDへ表示する（左=`0b1000`・前=`0b0110`・右=`0b0001`）。§2.65より前の、旋回中の前壁判定・連続旋回の半径の絞り・前壁なしの早出しなどは削除した（本フォークで追加、[スラローム機能_実装まとめ.md §2.25〜§2.36・§2.65](スラローム機能_実装まとめ.md)） |
| **10** | `mapErase()` | Flash上のマップ情報（`/map.txt`）をボタン操作のみで消去する。`g_map.mapClear()`で壁情報を未知状態（外周・スタート周辺のみ既知）に戻し`mapWrite()`で書き戻す。完了を`buttonOk()`の音で通知。モータは動かさない（本フォークで追加、2026-09-25に実車で消去を確認。詳細は[スラローム機能_実装まとめ.md §2.27](スラローム機能_実装まとめ.md)） |
| **15** | `motorDisable()` + `g_adjust.menu()` | モータ OFF して調整メニューへ移行 |
| 11〜14 | なし（`break` のみ） | 何もしない |

走行モード1〜9は、スタート区画から走り出す前に`startAccelerate()`で後ろ壁に当て、区画中心へ戻ってから加速する（本フォークで追加、[スラローム機能_実装まとめ.md §2.69](スラローム機能_実装まとめ.md)）。ゴールからの復路の開始では壁合わせをしない。

### モード1：左手法 `SEARCH::lefthand()`（`search.cpp:20`）

`accelerate(HALF_SECTION=90, search_speed)` で半区画進んでから、`while (switchGet() == 0)` のループに入る。ループ内の判定は次のとおり。

1. 左が空いている → 半区画減速 → 左90°旋回 → 半区画加速
2. 左に壁があり前方（FL/FR）が空き → 1 区画 `straight(SECTION=180, ...)`
3. 右が空いている → 半区画減速 → 右90°旋回 → 半区画加速
4. 全部壁 → 減速 → 右180°旋回 → 加速

**スイッチが押されるとループを抜け、`decelerate(HALF_SECTION, search_speed)` で半区画減速停止 → `modeExec` が戻り、`loop()` のモード選択へ復帰する**（本フォークでの変更、`search.cpp`）。判定はループ先頭（区画境界）でのみ行うため、直進・旋回の途中でボタンを押した場合は現在の区画動作を終えてから停止する。

### モード2：足立法 `SEARCH::adachi(gx, gy, gx2, gy2)`（`search.cpp:44`）

```text
positionInit()                     // 自己位置を (0,0)・北向きへ
adachi(goal_mx, goal_my)           // ゴールまで探索走行
rotate(right,2); nextDir(right)×2  // その場で180°回頭（実姿勢＋内部方向）
goalAppeal()                       // LED 演出＋ mapWrite()
adachi(0, 0)                       // スタートへ帰還（未探索区画を優先）
rotate(right,2); nextDir(right)×2  // 180°回頭
mapWrite()                         // 得られた迷路情報を Flash に上書き
```

`adachi()` 内部の 1 区画ごとの流れは次のとおり。

1. `nextDirGet()` で最初の進行方向へ回頭 → 半区画加速 → 内部座標更新
2. ゴールに着くまで次をループする。
   - `wallSet(sen_fr, sen_r, sen_l)` で現在区画と隣接区画の壁を記録
   - `nextDirGet()` が返す相対方向で走行
     - `front`：`oneStep(SECTION, search_speed)`（減速せず等速で1区画）
     - `right`/`left`：半区画減速 → 90°旋回 → 半区画加速
     - `rear`：半区画減速 → 180°旋回 → 半区画加速
   - 内部方向 `mypos.dir` 更新 → `axisUpdate()` で座標を1マス進める
3. ゴール到達で最後にもう一度 `wallSet()` → 半区画減速して停止

### モード4：迷路情報を取得する左手法 `SEARCH::lefthandGetWall(gx, gy, gx2, gy2)`（`search.cpp`、本フォークで追加）

`lefthand()` とは別関数。`misc.cpp` の `modeExec` case 4 は次のとおり。

```text
positionInit()                                  // 自己位置を (0,0)・北向きへ
lefthandGetWall(goal_mx, goal_my)               // 左手法で探索走行しながら壁を記録
mapWrite()                                       // 得られた迷路情報を Flash に保存
```

`lefthandGetWall()` 内部の流れ（`adachi()` と同じ自己位置管理方式）は次のとおり。

1. 北へ半区画加速 → `axisUpdate()` で最初の区画へ
2. 次をループする。
   - `wallSet(sen_fr, sen_r, sen_l)` で現在区画と隣接区画の壁を記録
   - **ゴール区画到達、またはいずれかのスイッチ押下で `break`**
   - 左手法で進行方向を決定
     - 左が空き → 半区画減速 → 左90°旋回（`nextDir(left)`）→ 半区画加速 → `axisUpdate()`
     - 前が空き → `oneStep(SECTION, search_speed)` → `axisUpdate()`
     - 右が空き → 半区画減速 → 右90°旋回（`nextDir(right)`）→ 半区画加速 → `axisUpdate()`
     - 行き止まり → 半区画減速 → 180°旋回（`nextDir(right)×2`）→ 半区画加速 → `axisUpdate()`
3. ループ脱出後、半区画減速停止 → `modeExec` が `mapWrite()` してモード選択へ戻る

- `lefthand()`（モード1）との違い：**自己位置 `mypos` を管理し、区画ごとに壁を `g_map` へ保存する**。前進は `straight` ではなく `oneStep`（等速で止めずに解析する足立法と同じ方式）。
- 左手法なのでゴールに到達しない迷路構造もあり得る。その場合はスイッチ押下で停止。
- 保存後はモード3（最短走行）やモード4[調整]（迷路確認）で情報を利用できる。

### モード5：左手法（スラローム版）`SEARCH::lefthandSlalom()`（`search.cpp`、本フォークで追加）

自己位置・壁情報を管理しないステートレスな左手法。90°旋回は`rotate()`を使う（`lefthand()`と同じ）。行き止まりのみ`uTurn()`（90度旋回→バック→前進×2で位置補正）を使う点が`lefthand()`との唯一の違い。

1. 左が空いている → 減速→`rotate(left,1)`→加速
2. 左に壁があり前方（FL/FR）が空き → 1 区画 `straight(SECTION=180, ...)`（モード1と同じ）
3. 右が空いている → 減速→`rotate(right,1)`→加速
4. 全部壁（行き止まり） → 減速→`uTurn(right)`→加速

終了条件・停止動作はモード1と同じ（スイッチ押下で次の区画境界まで走ってから半区画減速停止）。

**当初は90°旋回に`slalom(dir, 90)`を使っていたが、旋回開始位置の0.25区画早出し補正（§6参照）に必要な先読みができず実車で壁への衝突が頻発したため、本フォークで`rotate()`へ撤回**（自己位置・壁情報を管理しないため、次区画の壁が既知かどうかを判定する材料が無く、モード6のような先読み補正を組み込めない）。詳細は[スラローム機能_実装まとめ.md §2.16](スラローム機能_実装まとめ.md)参照。

### モード6：足立法（スラローム版）`SEARCH::adachiSlalom(gx, gy, gx2, gy2)`（`search.cpp`、本フォークで追加）

`misc.cpp` の `modeExec` case 6 はモード2（足立法）と同じ構成で `adachi()` を `adachiSlalom()` に差し替えたもの。

```text
positionInit()
adachiSlalom(goal_mx, goal_my)     // ゴールまで探索走行（スラローム版）
rotate(right,2); nextDir(right)×2
goalAppeal()
adachiSlalom(0, 0)                 // スタートへ帰還（スラローム版）
rotate(right,2); nextDir(right)×2
mapWrite()
```

`adachiSlalom()` は `adachi()` の左右90°旋回（`right`/`left`）だけを、区画境界から次の区画境界までを1動作で曲がる `slalomCell(dir)` に置き換えたもの（§6）。以下は`rotate()`のまま変更していない。

- 開始直後の最初の方向合わせ：まだ加速前で停止状態にあり、`slalom()`の前提（巡航速度で呼ぶこと）に合わないため
- `rear`（180°）：`HALF_SECTION`減速→`turnBack()`→`HALF_SECTION`加速。`turnBack()`は、前壁と左右どちらかの壁があれば位置補正付きの`uTurn()`（左壁があれば`uTurn(right)`、なければ`uTurn(left)`）、なければその場で`rotate(right, 2)`する。行き止まりでない区画でも足立法が後ろを選ぶことがあり、そのとき当てる壁がないままバックするのを防ぐため（[スラローム機能_実装まとめ.md §2.70](スラローム機能_実装まとめ.md)）

§2.65より前は、次区画の壁が既知なら直進を0.25区画早く切り上げる先読み（`peekKnownNextDir()`）を持っていたが、`slalomCell()`は区画境界から呼べばよいので削除した（[スラローム機能_実装まとめ.md §2.16・§2.65](スラローム機能_実装まとめ.md)）。

### モード3：最短走行 `FAST::run(gx, gy, gx2, gy2)`（`fast.cpp:20`）

```text
mapCopy()                          // Flash から迷路情報を読み込み
positionInit()
g_fast.run(goal_mx, goal_my)       // ゴールへ最短
rotate(right,2); nextDir(right)×2
goalAppeal()
adachiDynamic(0, 0)                // スタートへ（モード8の探索で戻る）
rotate(right,2); nextDir(right)×2
mapWrite()                         // 復路で得た迷路情報を保存
```

- `nextDir2Get()`（＝`map2Make`）を使い、**壁なしが確定（NOWALL）した経路のみ**で歩数マップを作る
- 直進が続く間は `straight_count++` でカウントし、曲がり角で `straight(straight_count*SECTION, search_speed, max_speed, search_speed)` として**連続直進区間を `max_speed`（既定1000）で一気に走る**
- 曲がり角・ゴールの前後の速度は、半区画（90mm）で止まり切れる`RUN::stoppableSpeed(HALF_SECTION)`（`search_speed`と`√(MIN_SPEED² + 2000 × search_accel × 90)`の小さい方）に抑える。減速し切れずに`stop()`で急停止するのを防ぐため。モード8の巡航速度も同じ（[スラローム機能_実装まとめ.md §2.58](スラローム機能_実装まとめ.md)）
- 復路は既知の経路ではなく足立法の探索で戻る（未知の壁を「壁なし」とみなすので未知区画を通る近い経路を選ぶ）。戻ったら`mapWrite()`する（[スラローム機能_実装まとめ.md §2.62](スラローム機能_実装まとめ.md)）
- ゴール区画へ入る区画境界で`wallSet()`し、ゴール区画の壁を記録してから止まる（`runSlalom()`も同じ）。複数区画のゴールでは探索で見ていない壁が残ることがあり、復路の探索が最初の向きを未知の壁で決めて衝突するのを防ぐため（[スラローム機能_実装まとめ.md §2.70](スラローム機能_実装まとめ.md)）

### モード7：最短走行（スラローム版）`FAST::runSlalom(gx, gy, gx2, gy2)`（`fast.cpp`、本フォークで追加）

`misc.cpp` の `modeExec` case 7 はモード3（最短走行）と同じ構成で `run()` を `runSlalom()` に差し替えたもの。

```text
mapCopy()                          // Flash から迷路情報を読み込み
positionInit()
g_fast.runSlalom(goal_mx, goal_my) // ゴールへ最短（スラローム版）
rotate(right,2); nextDir(right)×2
goalAppeal()
adachiSlalomDynamic(0, 0)          // スタートへ（モード9の探索で戻る）
rotate(right,2); nextDir(right)×2
mapWrite()
```

`runSlalom()` は `run()` と同じ骨組みで、曲がり角では直進を`straight(…, 終了速度 slalom_speed)`で終え、`slalomCell(dir)`（§6）で区画境界から次の区画境界まで曲がる。同方向・逆方向の連続旋回も`slalomCell()`を続けて呼ぶだけで、§2.65より前の先読み（`peekNextDir()`）・S字の角度の絞り・フック/U字の`slalom180()`・0.25区画早出しは削除した（[スラローム機能_実装まとめ.md §2.15・§2.43・§2.65](スラローム機能_実装まとめ.md)）。ゴール・行き止まりの前の直進は`stoppableSpeed(HALF_SECTION)`で終える。以下のみ`rotate()`のまま。

- **開始直後の最初の方向合わせ**：まだ加速前で停止状態にあり、`slalom()`の前提（巡航速度で呼ぶこと）に合わないため（`adachiSlalom()`と同じ理由）

`rear`（行き止まり）は**壁なしが確定した経路のみを使う最短走行では通常発生しない想定**の分岐。`runSlalom()`では`decelerate(HALF_SECTION) → turnBack() → accelerate(HALF_SECTION)`で処理する（`turnBack()`はモード6と同じ、[スラローム機能_実装まとめ.md §2.14・§2.43・§2.70](スラローム機能_実装まとめ.md)）。

### モード8：足立法（センサ閾値主導の動的到達判定版）`SEARCH::adachiDynamic(gx, gy, gx2, gy2)`（`search_dynamic.cpp`、本フォークで追加）

`adachi()`（`search.cpp`）と同一の骨格を持つ新規ファイル`search_dynamic.cpp`の関数。`misc.cpp`の`modeExec` case 8はモード2・6と同じ構成で`adachi()`を`adachiDynamic()`に差し替えたもの。`adachiDynamic()`が`adachi()`と異なるのは`case front`のみで、`oneStep(SECTION, search_speed)`の代わりに`run_dynamic.cpp`の新規関数`RUN::oneStepDynamic(max_len, init_speed, th_sum, debug_log)`を呼ぶ。`oneStepDynamic()`は距離カウント(`max_len`、壁が無い区画でのフォールバック)に加え、前方センサの生値の和`sen_fr + sen_fl`が閾値`th_sum`（モード8は`g_sensor.th_approach_sum`）を超えたら距離未達でも停止する。当初はFR/FL個別の閾値を両方超えたら停止していたが、車体の向きで停止位置がばらつくため和に変更した（§2.46）。既存の`th_wall`は区画中心で静止した状態で校正された値で、走行中の停止判定に転用すると早停止するため、別閾値にした。スタート区画から出るときは、開始時の`RUN::startAccelerate()`（`run_dynamic.cpp`、本フォークで追加）が`alignStartPosition()`で後ろ壁に押し付けてから`wall_return_len`だけ前進し、スタート位置を固定する（以前は`case 8`の先頭で呼んでいた、[スラローム機能_実装まとめ.md §2.69](スラローム機能_実装まとめ.md)）。閾値は調整モード8`straightSensorDiag()`（§5参照）で実測して決める。最初の助走は`SECTION / 3`で、`HALF_SECTION`との差(30mm)をループ最初の距離基準の動作に足して区画境界に合わせる（§2.47）。個別閾値の時点で実車での複数区画通し走行を確認済み。和判定（初期値290）の実車確認は未実施。`run.cpp`/`search.cpp`本体・既存モード1〜7は無変更。詳細は[スラローム機能_実装まとめ.md §2.18〜§2.24・§2.46・§2.47](スラローム機能_実装まとめ.md)参照。

### モード9：足立法（センサ閾値主導の動的到達判定版・スラローム版）`SEARCH::adachiSlalomDynamic(gx, gy, gx2, gy2)`（`search_dynamic.cpp`、本フォークで追加）

`adachiDynamic()`（モード8）と同じ骨格を持つ`search_dynamic.cpp`の関数。`misc.cpp`の`modeExec` case 9はモード8と同じ構成（スタート位置の固定も同じく`startAccelerate()`で行う）。モード8との違いは次のとおり（§2.65で現在の形に作り直した）。

- `right`/`left`：停止・`rotate()`せず、区画境界から`slalomCell(dir)`（§6）で曲がる。最初の助走（`SECTION / 3`）直後に曲がる場合は、不足分を`oneStep()`で足してから曲がる
- `front`：`oneStepDynamic(SECTION, search_speed, th_approach_sum_slalom)`。区画境界で判定するのはモード8と同じだが、モード9は`stoppableSpeed()`で抑えない`search_speed`で走るため、しきい値はモード9専用（Web UI`FRONT Approach Threshold(Slalom, FL+FR)`）にしてある
- 側壁：`sen_r/l.value`とモード9専用の`th_wall_slalom`（Web UI`SIDE Threshold(Slalom)`）を直接比べる（[スラローム機能_実装まとめ.md §2.28](スラローム機能_実装まとめ.md)）
- `rear`：`HALF_SECTION`減速→`turnBack()`→`HALF_SECTION`加速（モード6と同じ。[スラローム機能_実装まとめ.md §2.29・§2.70](スラローム機能_実装まとめ.md)）

§2.65より前は、早めに止まって旋回に入る方式だったため、旋回中の前壁判定（`slalomProbe()`）、連続旋回の角度の絞り・半径の絞り（`slalom_u_diff_max`など）、前壁なしの早出し（`slalom_open_cut`）、ゴール進入時の停止位置延長（`GOAL_ENTRY_EXTRA_LEN`）を持っていた。経緯は[スラローム機能_実装まとめ.md §2.25〜§2.54](スラローム機能_実装まとめ.md)参照。

### モード10：Flash上のマップ情報消去 `mapErase()`（`Flash.cpp`、本フォークで追加）

`misc.cpp`の`modeExec` case 10はモータを動かさずFlash上の迷路情報（`/map.txt`）だけを消去する。`Flash.cpp`の`mapErase()`が`g_map.mapClear()`（`MapManager`、コンストラクタの壁情報初期化ロジックを`public`メソッドとして切り出したもの）で壁情報を未知状態（外周・スタート地点周辺のみ既知、他は`_UNKNOWN`）に戻し、`mapWrite()`で`/map.txt`へ書き戻す。完了は`g_misc.buttonOk()`の確認音で通知。他モードと同じ「左右で選択→中央で実行」の操作感に統一するため、左右スイッチ同時押しのような新しい入力方式は使わず専用モード番号を割り当てた。現在位置・ゴール座標（`goal_mx`/`goal_my`）は変更しない。実行後にモード3/7（最短走行）を使うと、学習前の状態から`mapCopy()`されることになる。2026-09-25に実車でマップ消去を確認済み。詳細は[スラローム機能_実装まとめ.md §2.27](スラローム機能_実装まとめ.md)参照。

### ゴールアピール `MISC::goalAppeal()`（`misc.cpp:55`）

`motorDisable()` → `mapWrite()` → LED が左右に流れるアニメーション（間隔を徐々に短縮、10 回）→ `motorEnable()`。

---

## 5. 調整モード（`adjust.cpp` `ADJUST::menu` / `modeExec`）

モード15実行後、`menu()` が LED3 を 33 ms 間隔で点滅させながら（走行モード選択と区別）調整番号を選ぶ。中央スイッチで実行、`modeExec` が `1` を返すと（未定義番号、現在は10）走行モード選択へ戻る。調整番号を選んでいる間も、USB シリアルからのパラメータ設定コマンドを受け付ける（本フォークで追加、§9）。

| 番号 | 実プロジェクトの動作 | 備考 |
| --- | --- | --- |
| **1** | `webServerSetup()` | センサ値確認・パラメータ変更用 Web サーバ起動（AP: `192.168.4.1`）。ピッ×2 の後に起動 |
| **2** | `rotationCheck()` | **右90°旋回×8（計720°）→ トレッド調整**（旋回前後で向きが揃うよう `TREAD_WIDTH`）（本フォークで 3→2 に繰り上げ） |
| **3** | `circuitCheck()` | **4x4コースの最外周を直進→`slalomCell()`の繰り返しで周回**。各辺は2区画の直進と角の`slalomCell()`。つなぎ・脱調の有無と、旋回位置が区画に収まるかを確認する。角を曲がった後は、旋回で外輪が余計に走った距離`slalom_tail`をLEDの点灯数で表す（1個＝5mm未満で正常、4個＝45mm以上、§2.71）（本フォークで新規追加） |
| **4** | `straightToWallCheck()` | **前壁が見えるまで直進して区画中心で停止 → 姿勢制御 P ゲイン調整**（壁際に置き 3〜4 区画で中央に収束するよう `Gain` を調整）（本フォークで 2→4 に繰り下げ。区画数を数えない方式は[スラローム機能_実装まとめ.md §2.68](スラローム機能_実装まとめ.md)） |
| **5** | `straightCheck(3)` | **3 区画直進 → タイヤ直径調整**（目標より手前なら `TIRE_DIAMETER` を小さく）（番号は変更なし） |
| **6** | `adcView()` | **調整モード1の adcView 版**。シリアルモニタに r/fr/fl/l センサ値・VDD・`sample_reject`・TMC5240診断（`GSTAT`/`DRV_STATUS`）を連続表示（`webServerSetup()` の代わり。ピッ×2 の後に開始、**いずれかのスイッチで調整メニューへ戻る**）（本フォークで追加） |
| **7** | `slalomSequenceCheck()` | **連続旋回(S字)・旋回直後の直進・U字を一続きに確認**（本フォークで新規追加、詳細は下記） |
| **8** | `straightSensorDiag(2.5)` | **モード8・9の到達判定閾値の実測用診断**。既知の距離に置いた壁に向けて`oneStepDynamic()`を走らせ、走行距離とセンサ生値・その和をSerialへ出力する。実行中はセンサによる停止を無効化し、`section_check`（区画数、壁の距離に応じて要調整）の距離まで実測できる。壁への接触回避用にスイッチ中断にも対応（本フォークで新規追加、詳細は下記） |
| **9** | `slalomTurnCheck(right)` | **単発のスラローム旋回の確認**。モード9と同じ入り方（スタート位置合わせ→2区画直進→区画境界で`slalomCell()`）で1回曲がり、次の区画の中心で止まる。`slalom_offset`をSerialへ出力する（§2.65で旧`slalomSensorDiag()`から置き換え） |
| **10** | `frontThresholdCalib()` | **前センサの壁有無しきい値の校正**。区画中央に置き、1区画先（次の区画の奥）の前壁あり→なしの順に中央スイッチで測り、平均を`th_sen_fr`/`th_sen_fl`へ保存する（本フォークで追加、詳細は下記） |
| 11〜 | `return 1` | 走行モード選択へ戻る |

> **マニュアル p.71 の調整モード表・および過去バージョンの本ドキュメントの割り当てと実コードがずれている。**
> マニュアル表では「2=タイヤ直径、3=トレッド、4=迷路情報、5・6=なし、7=戻る」だが、
> 実プロジェクトでは **2 がトレッド調整（720°旋回）、3 がスラローム確認（新規）、4 が P ゲイン調整（前壁まで直進）、5 がタイヤ直径調整（3 区画）** に割り当てられている。
> 旧構成の `mapCopy()`＋`mapView()`（迷路情報のシリアル表示）は、本フォークの並び替えで `modeExec()` から呼ばれなくなり、2026-09-26 に `ADJUST::mapView()` 関数ごと削除した（[スラローム機能_実装まとめ.md §2.57](スラローム機能_実装まとめ.md)）。迷路情報は Web UI の `Maze Map` で確認できる。
> （マニュアル本文 p.104–105 は調整モード2を9区画のゲイン調整として説明しており、こちらとも一致しなくなった点に注意）。

- 調整モード3・4・5・7・8・9の走り出しは`startAccelerate()`で、後ろ壁に当てて区画中心に合わせてから加速する（[スラローム機能_実装まとめ.md §2.69](スラローム機能_実装まとめ.md)）。開始区画の後ろに壁があることが前提
- `straightCheck(n)`（`adjust.cpp:118`）：`accelerate(90)` → `n>1` なら `straight(180*(n-1), search_speed, max_speed, search_speed)` → `decelerate(90)`
- `rotationCheck()`（`adjust.cpp:133`）：`rotate(right, 1)` を8回繰り返し（720°）
- `circuitCheck()`（`adjust.cpp`、本フォークで新規追加、旧`slalomCheck()`を上書き）：`accelerate(90, search_speed)` → `do { straight(180*2, speed, search_speed, slalom_speed) → slalomCell(right) → ledSet(slalomTailLed(slalom_tail)) } while (switchGet()==0)` → `decelerate(90, ...)`。4x4コースの最外周（1辺3区画）を回る。スイッチの判定は角を曲がった後だけ
- `adcView()`（`adjust.cpp`）：Web サーバでセンサ値が不安定なとき用。**調整モード6**で呼び出す。シリアルモニタ（Tera Term 等、エスケープシーケンスで画面クリア）に r/fr/fl/l センサ値・VDD・`sample_reject`（FR/FL/R/L）・TMC5240の`GSTAT`/`DRV_STATUS`（左右）を100 ms 周期で表示。`while (switchGet() == 0)` ループなので**スイッチ押下で `menu()` へ復帰**（本フォークでの変更）。`GSTAT`/`DRV_STATUS`は「起動直後は探索速度が出ないが最短走行モードを一度実行すると直る」現象の切り分け用（§7 参照）。**バグ修正（2026-09-23）**：画面クリアのエスケープシーケンスが`\x1b[2j`（小文字j、未定義で多くの端末が無視する）になっていたため実際には画面がクリアされず、前フレームの表示に新フレームが重なって表示が乱れる不具合があった。`\x1b[2J`（大文字J、ED=Erase in Display）に修正（`mapView()`側の同じ誤りも合わせて修正）
- `slalomSequenceCheck()`（`adjust.cpp`、本フォークで新規追加）：**調整モード7**。スタートから 右→左→1区画直進→右→右 と曲がれるコースで行う
  1. `accelerate(HALF_SECTION, slalom_speed)` で区画境界まで助走
  2. `slalomCell(right)` → `slalomCell(left)`（S字）
  3. `straight(SECTION, g_run.speed, search_speed, slalom_speed)`（旋回直後の直進）
  4. `slalomCell(right)` → `slalomCell(right)`（U字）
  5. `decelerate(HALF_SECTION, g_run.speed)` で区画中心に停止
  各フェーズをLEDパターンで表示（直進=`0b0001`、S字=`0b0010`、U字=`0b0100`）
- `straightSensorDiag(section_check)`（`adjust.cpp`、本フォークで新規追加）：**調整モード8**。閾値に`INT_MAX`を渡してセンサ側の早期停止を無効化し、`accelerate(HALF_SECTION, search_speed)`で助走後`oneStepDynamic(SECTION*section_check, search_speed, INT_MAX, debug_log=true)`を呼んで、走行距離とセンサ生値(`sen_fr`/`sen_fl`)とその和(`sum`)を約20ms間隔でSerialへ出力する。既知の距離に置いた壁に向けて走らせ、モード8・9の到達判定閾値`th_approach_sum`/`th_approach_sum_slalom`（`sensor.h`、Web調整可能）を決めるための実測用。`section_check`は「壁の何区画手前で安全に止めるか」を表す実数（`float`）で、壁までの距離に応じて呼び出し側（現在は`modeExec()`のcase 8）を調整する必要がある。現在の`2.2`（396mm）は2026-09-23の実測に基づく値。スタート位置（後ろ壁に接触した機体の先端）から壁まで580mmで、`len`は`accelerate(HALF_SECTION)`の90mm先から数えるため壁まで490mm、半区画手前の目標停止点は`len`=400mm（2.22区画）になる。ただし2.5区画（450mm）で衝突したことがあり、エンコーダ距離と実距離に数十mmのずれがあるとみられるため、余裕を取って2.2区画にした。走行中にスイッチを押すと`oneStepDynamic()`側が即`stop()`し、後続の`decelerate()`もスキップされる（壁への接触回避用の安全弁）。スキップの判定は`oneStepDynamic()`の戻り値で行う（以前は`switchGet()`を呼び直していたため0が返り、止めた後に`decelerate()`が動いていた、[スラローム機能_実装まとめ.md §2.68](スラローム機能_実装まとめ.md)）。当初`th_approach`の無効化なしで実装したところ、閾値自体（未校正の仮値）で実測が早期に打ち切られ壁近傍のカーブが取れない不具合があり、早期停止の無効化・スイッチ中断を追加した（詳細は[スラローム機能_実装まとめ.md §2.21](スラローム機能_実装まとめ.md)、元機能は[§2.18](スラローム機能_実装まとめ.md)、和判定は[§2.46](スラローム機能_実装まとめ.md)参照）
- `straightToWallCheck()`（`adjust.cpp`、本フォークで新規追加）：**調整モード4**。`startAccelerate(HALF_SECTION, cruise_speed)`の後、`oneStepDynamic(SECTION*MAZESIZE_MAX, cruise_speed, th_approach_sum, debug_log=true)`で前壁が見えるまで直進する。`cruise_speed`は走行モード8と同じ`stoppableSpeed(HALF_SECTION)`。FR+FLが`th_approach_sum`を超えたところを区画境界とみなし、`decelerate(HALF_SECTION)`で区画中心に止まる。走行中は`len/fr/fl/sum`をSerialへ出し、スイッチで止められる。当初は前壁の判定と同じ`th_sen_fr`+`th_sen_fl`を使っていたが、1区画手前で止まったため変更した（[スラローム機能_実装まとめ.md §2.68](スラローム機能_実装まとめ.md)）
- `frontThresholdCalib()`（`adjust.cpp`、本フォークで新規追加）：**調整モード10**。区画中央に置き、その区画には前壁を置かず、1区画先（次の区画の奥、約270mm先）に壁を置いてLED`0b0110`で中央スイッチを押す。確認音の後、壁を外してLED`0b1001`で中央スイッチを押す。それぞれ0.5秒待ってからFR/FLを約1秒（100回）平均し、`Sensor ratio`で割ったベース値を`sen_fr/fl_wall`・`sen_fr/fl_nowall`に、その平均を`th_sen_fr`/`th_sen_fl`に入れてFlashへ保存する。中央以外のスイッチで中止する（保存しない）。2026-10-22の実測は壁あり54/37、なし10/3、しきい値32/20（[スラローム機能_実装まとめ.md §2.67](スラローム機能_実装まとめ.md)）

---

## 6. 走行制御の仕組み（`run.cpp`）

### 速度更新割り込み `RUN::interrupt()`（1 kHz）

1. `speed += accel` を毎周期加算し、`upper_speed_limit` / `lower_speed_limit` でクリップ
2. `con_wall.enable == true` のとき**壁 P 制御**（旋回中は `false`＝姿勢制御しない。旋回中に壁センサが反応すると目標角度まで回れないため）
   - 両壁が制御範囲内：`error = sen_r.error - sen_l.error`
   - 片壁のみ：`error = 2.0 * (sen_r.error - sen_l.error)`
   - `control = 0.001 * speed * kp * error`（`0.001` は割り込み周期 1 ms、速度比例にして低速でも高速でも同じ収束感）
   - `speed_target_r = speed + control` / `speed_target_l = speed - control`
   - 下限 `MIN_SPEED(30)` を下回らないよう補正
   - ※ `con_wall.diff` / `con_wall.sum`（D・I 項の下地）は計算しているが**実際には kp のみ使用の P 制御**
3. `con_wall.enable == false` かつ `turn_enable == true` のとき**スラローム用の左右速度制御**（本フォークで追加、§2.65で現在の形に変更）
   - 外輪 = `speed × (1 + slalom_outer_gain × turn_f)`、内輪 = `speed × (1 − (1 − slalom_inner_ratio) × turn_f)`（`turn_sign`：左旋回 +1・右旋回 −1）
   - 旋回の強さ `turn_f`（0〜1）は`slalom()`の旋回ループが外輪の走行距離から更新する
   - 内輪が `MIN_SPEED` を下回るときは、左右に同じ倍率（上限`SLALOM_MIN_SPEED_SCALE_MAX` 2.0）を掛けて速度比を保つ。下限クランプで比がずれると、旋回の出口が前に出る（詳細は[スラローム機能_実装まとめ.md §2.71](スラローム機能_実装まとめ.md)参照）
   - どちらのブランチにも該当しない場合（旋回もスラロームもしない）は `speed_target_r = speed_target_l = speed`

### 主要走行関数

| 関数 | 動作 |
| --- | --- |
| `straight(len, init, max, finish)` | 加速→（残距離が制動距離を切ったら）減速。`finish==MIN_SPEED` なら `stop()`、それ以外は `stay(finish)` で等速継続。姿勢制御 ON |
| `accelerate(len, finish)` | `MIN_SPEED` から `finish` まで加速、`stay()` で終える |
| `oneStep(len, init)` | 加減速なし等速で 1 区画。`counterClear()` しないので前区間のパルスを引き継ぐ |
| `oneStepDynamic(max_len, init, th_sum, debug_log)` | **本フォークで追加（`run_dynamic.cpp`）**。`oneStep()`と同じ等速走行だが、`max_len`の距離カウント(壁が無い区画でのフォールバック)に加え、前方センサの生値の和`sen_fr + sen_fl`が`th_sum`を超えたら距離未達でも停止する。和にするのは車体の向きの影響を打ち消すため（§2.46）。`th_wall`は静止校正値のため走行中の停止判定には転用できず、別閾値を新設（`調整モード8`で実測して決める）。`debug_log=true`で走行距離とセンサ生値・和をSerialへ出力し（実測用）、スイッチで中断できる。中断したら`true`を返す（§2.68）。既存の`oneStep()`は無変更。詳細は[スラローム機能_実装まとめ.md §2.18・§2.46](スラローム機能_実装まとめ.md)参照 |
| `startAccelerate(len, finish, align)` | **本フォークで追加（`run_dynamic.cpp`）**。走行開始時の加速。`align=true`なら`alignStartPosition()`（後ろ壁へ`uturn_back_len`後進→`wall_return_len`前進）で区画中心に合わせてから`accelerate(len, finish)`。往復で使う探索関数は、現在位置がスタート区画(0,0)のときだけ`align=true`にする（復路の開始は後ろに壁が無い）。詳細は[スラローム機能_実装まとめ.md §2.69](スラローム機能_実装まとめ.md)参照 |
| `decelerate(len, init)` | 減速して `stop()`。`counterClear()` しない |
| `rotate(dir, times)` | 左右モータ逆回転で超信地旋回。`times`×90°。`tread_width*π/4*times` の弧長で加減速。姿勢制御 OFF。**停止済みの状態から呼ぶ前提**（`speed`を`MIN_SPEED`にリセットして開始し、終了時に`stop()`）。**本フォークで`TURN_TIMEOUT_MS`タイムアウトを追加**：基板接触不良でパルス数が更新されなくなっても`while(1)`が終わらなくならないよう、経過時間超過で強制`stop()`する（詳細は[スラローム機能_実装まとめ.md §2.17](スラローム機能_実装まとめ.md)参照） |
| `slalom(dir, angle)` | **本フォークで追加、§2.65で方式を変更**。左右モータは同方向のまま、外輪を増速・内輪を減速して**停止せずに**曲がる。`turn_f`は外輪の走行距離に対する前後対称な台形で、入口と出口はそれぞれ`slalom_ramp_len`[mm]で0⇔1。台形全体の長さは開始時に`slalomLength(angle)`で求めるので、任意の角度で前方向と横方向の張り出しが等しくなる（[スラローム機能_実装まとめ.md §2.66](スラローム機能_実装まとめ.md)）。軌跡は外輪の走行距離で決まり、速度にも旋回中の減速にもよらない。並進速度は旋回中に`slalom_speed`まで下げ、そのまま次の動作へつなげる。旋回角は`(右輪距離 − 左輪距離) / tread_width`で判定。`TURN_TIMEOUT_MS`のタイムアウトあり。**末尾で`counterClear()`する**（[スラローム機能_実装まとめ.md §2.9](スラローム機能_実装まとめ.md)） |
| `slalomCell(dir)` | **§2.65で追加**。区画境界から次の区画境界までを1動作で90°曲がる：`oneStep(HALF_SECTION − slalom_offset)` → `slalom(dir, 90)` → `oneStep(HALF_SECTION − slalom_offset)`。区画中心の前後`slalom_offset`で旋回する。連続旋回・U字は続けて呼ぶだけでよい |
| `slalomParamUpdate()` | **§2.65で追加**。スラローム形状パラメータを範囲内に収め、`slalom()`と同じプロファイルを数値積分して90°旋回の張り出し`slalom_offset`（区画中心から旋回の開始点・終了点までの距離）を求める。あわせて入口ランプで曲がる角度`slalom_ramp_th`を求め、Serialに横方向の張り出しも出す（対称性の確認用、§2.66）。`paramApply()`から呼ばれる |
| `stay(speed)` | 割り込みを止めて等速固定・`accel=0`・カウンタクリア。**迷路解析中もタイヤを止めずに転がし続ける**ための関数（止めるとロック→スリップして距離がずれる。解析中に進んだ分は次の走行関数で目標距離から差し引く。マニュアル p.88） |
| `stop()` | `VMAX=0`、`delay(300)` で惰性が消えるのを待つ |

- 距離 → パルス換算：`obj_step = len * 2.0 / pulse`（左右パルス和 `step_lr` で比較）
- `pulse = tire_diameter * π / (200 * 16)`（200 ステップ/回転 × 16 マイクロステップ）
- `slalom()`用に `TMC5240::readXactualLR()` で左右パルスを符号付きのまま個別取得（`step_l`/`step_r`）し、`step_l_len = step_l*pulse` / `step_r_len = step_r*pulse` に変換（`stepGetLR()`）。`readXactual()`（絶対値の和）は並進や超信地旋回のように「左右が常に同じ大きさで動く」場合にしか回転量・並進量を正しく表さないため、並進しながら回転するスラロームでは使えない
- `slalom()`は**呼び出し時点で`speed`が巡航速度(`MIN_SPEED`超)であることが前提**。停止状態(`speed=0`)から呼ぶと下限クランプで左右差が潰れ、正しく曲がらない
- **本フォークで追加**：`slalomCell()`を`adachiSlalom()`（モード6）・`runSlalom()`（モード7）・`adachiSlalomDynamic()`（モード9）が使う。`lefthandSlalom()`（モード5）は`rotate()`のまま。行き止まりはいずれも位置補正付きの`uTurn()`

### 解決済み：`slalom()`直後の`straight()`が短く終わる → `slalom()`のカウンタクリア漏れ（2026-09-18）

**症状**：`circuitCheck()`（4x4コース周回、§4§5）を実車で調整していたところ、単純に「角ごとに`straight(SECTION*3, ...) → slalom(right,1)`を繰り返す」設計では曲がるタイミングが想定よりずれ、実測に合わせて`straight(SECTION*2.75, ...)`のような半端な値でなければ辻褄が合わなかった。

**調査**：`run.cpp`の各走行関数の`counterClear()`（走行パルスカウンタのリセット）の有無を洗い出したところ、`straight()`/`oneStep()`/`decelerate()`はいずれも**自分の開始時にはクリアしない**設計（`stay()`で等速のまま「考え中」に転がった分の距離を、次の走行関数の目標距離から自動的に差し引くための意図的な仕組み。マニュアル p.88、`stay()`の項参照）。この前提は「直前の関数が意味のある基準点でカウンタをクリア済み」であることに依存しており、`accelerate()`/`oneStep()`は`stay()`経由で終了時に必ずクリア、`rotate()`は終了時にクリアしないが直後には必ず`accelerate()`が呼ばれる（`decelerate→rotate→accelerate`という既存の並びで一貫して守られていた）ため問題にならなかった。**`slalom()`だけが「速度を落とさず次につなげる」ために`stop()`も`stay()`も呼ばず終了時にカウンタをクリアしていなかった**ため、直後の`straight()`が旋回中に溜まった生パルスを「すでに走った距離」として引き継ぎ、渡した`len`より短く終わっていた。

**修正**：`RUN::slalom()`の末尾に`counterClear()`を追加（`run.cpp`）。これにより`slalom()`の直後の`straight()`/`oneStep()`/`decelerate()`には素の距離（そこから何区画分か）を渡せばよくなった。

**影響**は次のとおり。

- `lefthandSlalom()`/`adachiSlalom()`（`search.cpp`）・`slalomSequenceCheck()`（モード7）は元々「素の1区画」のつもりで`SECTION`を渡していたため、コード変更は不要（今まで短く終わっていたバグがこの修正で解消される）
- `circuitCheck()`（モード3）はカウンタが残る前提で経験的に`SECTION*2`/`SECTION*2.75`へ調整していたため、修正後の意味に合わせて理論値`SECTION*2.25`/`SECTION*2.5`へ更新し、その後ユーザー側で実車再調整して`SECTION*2`（初回）／`SECTION*1.75`（2辺目以降）で**調整完了**（2026-09-18）

詳細は[スラローム機能_実装まとめ.md §2.9](スラローム機能_実装まとめ.md)参照。

### 解消：連続旋回で横方向に0.5区画ずれる現象 → `slalom()`に角度パラメータを追加（2026-09-18）→ §2.65で`slalomCell()`に置き換え

§2.65で旋回が区画境界から区画境界の動作（`slalomCell()`）になり、連続旋回で角度を絞る必要はなくなった（`slalom_multi_angle`は削除）。以下は経緯として残す。

**症状**：`slalom()`のカウンタクリア修正後、`slalomSequenceCheck()`（モード7）の連続旋回フェーズ（直進を挟まず`slalom(right,1)`→`slalom(left,1)`）を実車確認したところ、2回旋回した後の位置が横方向に0.5区画ずれていた。旋回前後の直進が影響していると推定。

**対策**：`RUN::slalom(dir, times)`の`times`（90度単位の整数倍しか指定できない）を`angle`（度単位の`float`、任意角度を指定可能）に一般化した（`obj_turn = tread_width*π*angle/360`、`angle=90`で従来の`times=1`と一致）。直進を挟まない連続旋回では90°+90°だと横方向のズレが大きくなりすぎるため、新パラメータ`slalom_multi_angle`（初期値80度、Web調整可能）で90°未満に減らして曲がれるようにし、`slalomSequenceCheck()`の連続旋回フェーズに適用した。単発の90°旋回（`lefthandSlalom()`/`adachiSlalom()`・`circuitCheck()`）は`slalom(dir, 90)`とリテラル値で呼ぶよう変更（動作自体は変わらない）。

**未対応**：実際の探索走行（`lefthandSlalom()`/`adachiSlalom()`）には未適用。これらは1区画ごとの反応的なアルゴリズムで「次も旋回するか」を事前に知る先読みロジックがなく、適用するには設計が必要。`slalom_multi_angle`（当初80度→75度に実車調整、継続検証中）自体の検証もまだ途中。実運用への適用方針（同方向/逆方向で扱いを分ける必要がある点、行き止まりの`uTurn()`は置き換え不可な点など）は実装せず設計のみ検討済み。詳細は[スラローム機能_実装まとめ.md §2.10・§2.11](スラローム機能_実装まとめ.md)参照。

### 解決済み：旋回・短距離動作が遅い現象 → Flash（SPIFFS）に保存された古い/不正なパラメータ値（2026-09-15）

**症状**（当初の見え方）：電源投入後、最短走行モード（モード3）以外の走行モード（モード1/2/4）を実行すると探索速度自体が明らかに遅い。モード3を一度実行すると直ったように見えたが、その後の切り分けで「モードの違い」ではなく「**動作の種類**」の問題（モード3自身の中でも、長い直線区間`straight()`だけが速く、旋回・短距離動作は遅いまま）と判明。

**調査の過程**（結果的に的中しなかったが記録として残す）

- モード2を2回連続実行しても直らず → 「起動後最初の走行なら何でも直る」説を否定
- TMC5240の`GSTAT`/`DRV_STATUS`を比較 → リセット・ドライバエラー・充電ポンプ低電圧・過熱シャットダウン(`ot`/`otpw`)・StealthChop収束待ちは全て否定。`cs_actual`（実電流スケール）は走行直後28（`IRUN`とほぼ一致）で電流はフルに出ていた
- 左右のSG_RESULT（StallGuard生値）が非対称（L=30/R=80、値が小さいほど負荷大）で、手で車輪を回すと左側にわずかな抵抗を感じたため、**左車輪の機械的抵抗が主因と一旦結論づけていた**が、これは誤りだった（後述の通り真因はFlash側。左右差自体は実在する可能性はあるが、今回の主症状の説明にはならなかった）

**真の原因**：**Flashメモリ（SPIFFS `/parameters.txt`）を消去したところ症状が解消**。つまり、SPIFFSに保存されていたパラメータ（`search_accel`など、過去のWeb UIでの調整や以前のファームウェアバージョンとの互換性の問題で不正・過小な値のまま保存され続けていたと推測）が原因で、`paramRead()`がその値を読み込んで`g_run.search_accel`等に反映していたため、**直進系の加減速だけが極端に鈍かった**。`turn_accel`は別パラメータのため旋回だけは正常に見え、逆に一見「旋回の方が遅い」ように錯覚した。Flashを消す（`flashInit()`で`parameter.h`の初期値に戻る）ことで正しい`SEARCH_ACCEL_INIT`等が復元され、解消した。

**教訓・注意点**は次のとおり。

- `paramWrite()`は`webserver.cpp`でパラメータを変更（Save）するたびにSPIFFSへ上書きする。**Web UIで一時的に小さい値を試して保存すると、電源を切っても値が残り続ける**ため、意図せず低速化した状態がデフォルトの動作のように見えてしまう
- パラメータファイルの内容に心当たりがない不調が出たら、**まずFlashを消して`parameter.h`の初期値に戻して切り分ける**のが手早い（中央スイッチを押しながら起動 → `flashInit()`）
- 左車輪の手回し時のわずかな抵抗は今回の主因ではなかったが、気になるようであれば別途清掃・注油の価値はある

**再発防止（本フォークで追加）**：`paramRead()`（`Flash.cpp`）に速度系パラメータの妥当性チェックを追加した。`search_speed`/`max_speed`が`MIN_SPEED`未満または`SPEED_SANITY_MAX`（3000mm/s、`parameter.h`）超、`search_accel`/`turn_accel`が0以下または`ACCEL_SANITY_MAX`（100、`parameter.h`）超の場合、その値を**そのパラメータの`*_INIT`（`parameter.h`）へフォールバック**し、シリアルに警告を出すようにした。これにより、Flashにおかしな値や意図しない小さい値が残っていても、起動時に自動的に安全な初期値へ戻り、今回のような「探索速度だけ極端に遅い」という気づきにくい不具合が再発しにくくなる。

**今回追加した診断機能**（副産物だが引き続き有用、残置）：TMC5240の `GSTAT`（bit0=reset bit1=drv_err bit2=uv_cp）と `DRV_STATUS`（bit31=stst待機中 bit30=olb bit29=ola開放線 bit28=s2gb bit27=s2ga地絡 bit26=otpw高温警告 bit25=ot高温シャットダウン bit24=stallguard bit16-20=cs_actual実電流スケール bit14=stealth bit0-9=sg_result）を、`adcView()`（調整モード6）と各走行モード終了時ログ（`misc.cpp` `modeExec()` 末尾、`motorDisable()`前、**走行モード1〜8が対象**。モードの追加に合わせて対象範囲を1〜4→1〜6→1〜7→1〜8へ拡張）の両方から確認できる（`TMC5240.cpp` `TMC5240::read()`）。モーターの過熱・脱調・左右負荷差の今後の早期発見に使える。

---

## 7. センサ処理（`sensor.cpp` / `device.cpp`）

`sensorInterrupt()` は **core1 固定タスク `sensorTask`（`device.cpp`）から 1 ms 周期で 5 回**呼ばれる（旧: タイマ1の 4 kHz ISR）。`cnt` 0が電池電圧/LED処理（SLEDなし）、`cnt` 1→4 で FR / FL / R / L を 1 つずつ点灯・AD 読み取り・消灯（起床直後の枠にSLED点灯を伴わない処理を置く理由は下記「WiFi 接続中の…」参照）。

- `sensorGetX()` → `sledRead()`：`digitalWrite(SLED_x, HIGH)` → **`delayMicroseconds(SLED_SETTLE_US)`** → `analogRead()` → `LOW`。点灯時間を `micros()` 実測し `SLED_WINDOW_MAX_US` 超過なら `SENSOR_SAMPLE_BAD`（下記「WiFi 接続中の…」）
- 各センサ：`value > th_wall` で `is_wall = true`。ただし前壁は FR+FL の和で判定し（`(FR + FL) > (FRのth_wall + FLのth_wall)`）、`sen_fr.is_wall` と `sen_fl.is_wall` に同じ結果を入れる（[スラローム機能_実装まとめ.md §2.57](スラローム機能_実装まとめ.md)）
- 横センサ（R/L）：`value > th_control` のとき `error = value - ref`、`is_control = true`（姿勢制御に使用）。範囲外なら `error = 0`
- `cnt==0`（起床直後、SLEDなし）で電池電圧処理：`battery_value = batteryVoltGet()`、10 回中の点灯回数で青（`BLED0`）／赤（`BLED1`）2 色 LED の割合を変化（12.0 V 以上=青のみ、10.0 V 以下=赤のみ）。`BATT_MIN(10000)` 未満が `BATT_LOW_ALARM_MS` 連続したらブザー 400 Hz ＋ LED 全消灯（デバウンス、§「WiFi 接続時のブラウンアウト」参照）

### センサ値の安定化（フォークで追加）

壁センサ出力は SLED 立ち上がりで一瞬光る**減衰パルス**（LED を点けっぱなしにすると出力は消える）。素のファームは `for(300){nop}` ≒ 5µs 待ちで 1 回読むだけで、急峻な減衰部を `analogRead()` のレイテンシ揺れ（±10µs級）を伴って読むため、1 チャンネルが p-p 60〜100 で暴れていた（特に調整モードで顕著）。

| 変更 | 内容 |
| --- | --- |
| `parameter.h` | `WAITLOOP_SLED` 廃止 → `SLED_SETTLE_US = 32`（LED フラッシュ後の緩い RC 裾で読む＝ジッタ感度が最小）、`SEN_IIR_A = 3`、`SLED_WINDOW_MAX_US = 200`、`SENSOR_SAMPLE_BAD = 0xFFFF` |
| `device.cpp` | `sensorGetR/L/FL/FR()` の nop ループ → 共通 `sledRead()`：`digitalWrite(HIGH)` → `micros()` → `delayMicroseconds(SLED_SETTLE_US)` → `analogRead()` → 点灯時間を計測。**`SLED_WINDOW_MAX_US` を超えたら `SENSOR_SAMPLE_BAD` を返す** |
| `sensor.cpp` | 生読み → `SENSOR_SAMPLE_BAD` なら**棄却して前回値維持**（`sample_reject[ch]++`）→ **メディアン3**（単発スパイク除去）→ **IIR `value=(value*3+in)/4`**（TC≒3ms）。1 パルス 1 リードは維持 |
| `sensor.h` | `volatile unsigned long sample_reject[4]`（ch: 0=FR 1=FL 2=R 3=L）を追加。`adcView()` で表示 |

- 結果（WiFi 未接続）：4 ch 中 3 ch が p-p 0〜2、残り 1 ch（毎回異なる／物理要因の疑い）が p-p ~20。
- 信号振幅がピーク比で約 7 割に低下（この距離で FR≒235 / FL≒276 / R≒245 / L≒251）。**REF / Threshold の再取得が必要**（`mode 1` で現物合わせ）。
- 壁 P 制御は約 3ms 遅れた `error` を使う。実走（mode 2）で発振しないか要確認。ダメなら `SEN_IIR_A` を 1 に。
- 切り分けは `step8_test` の調整モード 6（`sensorDiag()`）で実施（settle スイープ／LED hold 特性／IIR シミュレーション）。

#### WiFi 接続中のセンサ値低下・ブレ（フォークで対策）

**症状**：旧構成はセンサ読みをタイマ割り込み（`onTimer1` 4 kHz）内で行い、`analogRead()`（非 IRAM・内部 mutex）を呼んでいた。web サーバに端末が接続すると WiFi 割り込み・キャッシュ停止で ISR が延び、**SLED 点灯窓（HIGH〜AD 確定）が数百µs〜ms に伸びて減衰パルスの下側を読む** → 値が系統的に低下しブレ増（シリアル `adcView` でも顕在化。未接続時は正常）。`SLED_WINDOW_MAX_US=200` の棄却だけでは差が残った（200µs 未満の伸びでも読み下がる／TX 時の電源ドロップ成分）。

**対策（3 段）** は次のとおり。

1. **センサ計測を core1 固定タスク `sensorTask` に移設**（`device.cpp`、旧 `onTimer1` を廃止）。
   - `xTaskCreatePinnedToCore(sensorTask, ..., prio 20, core 1)`。`vTaskDelayUntil` で 1 ms 周期、毎周期 5 回（電池電圧処理1回＋FR/FL/R/L4ch）を順に読む（各 ch 実効 1 kHz ＝従来同）。
   - WiFi/AsyncTCP は core0、センサは core1。**WiFi 割り込みが SLED 点灯窓を延ばせなくなる**。ISR ではなくタスクなので、キャッシュ停止時も「停止して待つ」だけでクラッシュしない（旧来 Flash アクセス前に割り込みを止めていた制約の主目的）。
   - `sensorInterruptStop()/Start()` は `g_sensor_run` フラグ制御に変更（`g_sensor_paused` で 1 サイクル完了を待って停止）。`Flash.cpp` からの呼び出しはそのまま有効。
2. **`sledRead()` の点灯時間棄却**（`micros()` 実測 > `SLED_WINDOW_MAX_US` → `SENSOR_SAMPLE_BAD`、前回値維持）。core1 内の残留擾乱（`controlInterrupt` ISR 等）や Flash 書き込み時のキャッシュ停止に対する保険。棄却数は `adcView` の `reject FR/FL/R/L`。`SLED_WINDOW_MAX_US` はまず広め（200 us）にして誤棄却ゼロを確認し、WiFi 未接続で reject がほぼ増えない下限まで詰める（目安：クリーン時の点灯時間（`SLED_SETTLE_US`＋`analogRead()` の数十 us）＋30〜50 us）。
3. **起床直後の枠をSLEDなし処理に差し替え**（`sensor.cpp`）。`sensorTask` が `vTaskDelayUntil` から起床した直後の1回目（`cnt==0`）は、core0のWiFi/AsyncTCP活動によるスケジューラ・割り込みジッタを最も受けやすい。ここに元々置かれていたFRの読みを電池電圧処理（SLEDなし）に差し替え、FR/FL/R/Lを`cnt==1〜4`にずらした。**実績：WiFi接続時にFRだけ突出していたreject（+2000程度）が対策後に大幅減少、ブラウザ/シリアルの値の乖離も改善**（2026-09-15）。

**残る注意点**は次のとおり。

- `controlInterrupt`（timer0 ISR, core1）は `sen_r/l.error` を排他なしで読む（旧 `g_timer_mux` 相当の保護なし）。更新は 1 ms 周期・r/l 約 60µs 差なので、ISR が両者更新の隙間に入る確率は小さく、入っても 1 周期の軽微なズレ（`error` は IIR で緩慢）。実走（mode 2）で発振しないか要確認。
- キャリブレーション値は引き続き **WiFi 未接続（`mode 6` 単独）のシリアル値**を基準に。

#### 解消済み：WiFi 接続時のブラウンアウト（`E BOD` / 誤 低電圧ブザー）（2026-09-15、バッテリ運用に切替）

WiFi 接続後に **`E BOD: Brownout detector was triggered` ＋ ブザー**、FR の reject 増 が出る場合、**WiFi TX 電流バースト（〜350mA ピーク）で 3.3V レールが落ちている**。ブラウンアウトでチップが一時停止／リセット → タスク復帰時の初回読み（＝ FR）が伸びて reject 増。電池 ADC も一瞬下がり `battery_value < BATT_MIN` で低電圧ブザーが誤発報。

- **最有力の原因は電源側**。USB 給電で試験している場合は**充電済みバッテリ**に切り替える（USB は WiFi TX の突入電流に足りないことが多い。ケーブル／ポートが弱いと特に）。バッテリも消耗していれば充電。可能なら 3.3V にバルクコンデンサ（470〜1000µF）追加。
- **firmware 緩和（実装済み）**
  - `webserver.cpp`：`WiFi.setTxPower(WIFI_POWER_8_5dBm)` を `softAP()`／`begin()` 後に設定（`WEB_WIFI_TX_POWER`）。TX バーストを抑える。近距離運用なので通信品質は問題なし。足りなければ `WIFI_POWER_5dBm` / `2dBm`。
  - `sensor.cpp`：低電圧ブザーを**デバウンス**（`BATT_LOW_ALARM_MS = 200`：連続 200ms 低下で初めて発報）。数ms の TX ディップでは鳴らない。
  - 追加でポーリング間引き（`webserver.cpp` `setInterval` 200→300〜500ms、`/map` 停止）も TX 頻度を下げる。
- **ブラウンアウト検出の無効化は非推奨**（低電圧での動作は無効化しても不安定・データ破損の恐れ。症状を隠すだけ）。

---

## 8. 迷路管理・歩数マップ（`map_manager.cpp`）

- 迷路サイズは `g_map.maze_x/maze_y`（2〜16、初期値は`parameter.h`の`MAZESIZE_X/Y_INIT`で STEP8 は練習用の 4×4、STEP9 は本番用の 16×16）。Web の `MAZE SIZE` で変更でき、`/parameters.txt` の `maze_x`/`maze_y` に保存する。変更して保存すると迷路情報を消去する。壁情報・歩数マップの配列は常に `MAZESIZE_MAX`（16）×16 で確保する（[スラローム機能_実装まとめ.md §2.59](スラローム機能_実装まとめ.md)）。`/map.txt` は常に 16×16 byte。壁は 1 区画 = ビットフィールド 1 byte（北/東/南/西 各 2 bit、`NOWALL=0 / WALL=1 / _UNKNOWN=2`）
- コンストラクタ：全区画 `_UNKNOWN` → 外周を `WALL` → スタート区画の東に壁・北は壁なし。`mypos=(0,0,north)`
- `wallSet()`：自己位置の向きに応じて現在区画と隣接区画の壁を同時に更新
- `searchMapMake(x,y,x2,y2)`（探索用）：`WALL 以外`（未知も通れる前提）で、目標の長方形 (x,y)-(x2,y2) 内の全区画を 0 とした歩数マップを波及計算（`stepMapInit()`、複数区画ゴール。スタートへ戻るときは (0,0,0,0)、[スラローム機能_実装まとめ.md §2.56](スラローム機能_実装まとめ.md)）
- `map2Make(x,y,x2,y2)`（最短用）：`NOWALL` が確定した辺のみで歩数マップを計算（目標の扱いは同上）
- `posIn(x,y,x2,y2)`：現在位置が目標の長方形内か。探索・最短走行の終了判定に使う
- `nextDirGet` / `nextDir2Get`：4 方向を調べ、`stepMapSet` で「歩数が最小」→ 同点なら `priorityGet` の高い方を選択
  - `priorityGet`：同方向 +2 / 逆方向 +0 / それ以外 +1、対象区画に未知壁があれば +4（**未探索方向を優先＝足立法の探索性**）
  - 目標へ到達不能（`steps_map == 65535`）なら LED を交互点滅（**本フォークでモータ停止＋スイッチでの脱出を追加**。衝突による基板接触不良で壁情報がおかしくなり誤発生することがあるため。スイッチで抜けた後は、往路・復路に関わらず`modeExec()`の残り（復路・ゴールアピール・`mapWrite()`）を行わずモード選択へ戻る。詳細は[スラローム機能_実装まとめ.md §2.17・§2.53](スラローム機能_実装まとめ.md)参照）

---

## 9. Flash 保存（`Flash.cpp`、SPIFFS）

- Flash アクセス中はタイマ割り込み禁止（ESP32 仕様）→ `controlInterruptStop()` / `sensorInterruptStop()` で囲む
- `mapWrite()` / `mapCopy()`：`/map.txt` に 16×16 byte のバイナリで壁情報を読み書き。実際の迷路サイズ(`maze_x/maze_y`)の範囲内のみ読み書きし、範囲外のセルは`mapWrite()`で0を書き`mapCopy()`で読み飛ばす（16×16固定でループして`wall`配列へ範囲外アクセスし、最短走行後に`mode_select`等がおかしくなっていた不具合の修正、§2.44）。ファイルを開けない場合も割り込みを再開してから`return`する
- `mapErase()`（本フォークで追加）：`g_map.mapClear()`で壁情報を未知状態（外周・スタート地点周辺のみ既知）に戻してから`mapWrite()`で`/map.txt`へ書き戻す。走行モード10からボタン操作のみで呼び出せる（詳細は[スラローム機能_実装まとめ.md §2.27](スラローム機能_実装まとめ.md)）
- `paramWrite()` / `paramRead()`：`/parameters.txt` に「キー 値\n」形式のテキストで保存。Web サーバで変更した値はここに保存される
  - パラメータは表`param_table`（キー・型・格納先・初期値・範囲）で管理する（[スラローム機能_実装まとめ.md §2.65](スラローム機能_実装まとめ.md)）。読込ではまず全パラメータを初期値にし、ファイルにあるキーだけ上書きする。範囲外の値は初期値に戻す。**未知のキー（削除したパラメータ）は無視する**（以前は全パラメータを初期化していた）
  - センサのREF・しきい値はベース値として保存し、判定には「ベース値 × `sen_ratio`[%]」を使う（`SENSOR::applyRatio()`、[スラローム機能_実装まとめ.md §2.63](スラローム機能_実装まとめ.md)）
  - `paramCommit(old_maze_x, old_maze_y)`（本フォークで追加、2026-10-22）：`paramSet()`で値を設定した後の処理を、Web と USB シリアルで共通にした関数。迷路サイズを検証し、サイズが変わったら`mapErase()`する。続けてゴール座標を検証し、`paramApply()`→`paramWrite()`の順に反映・保存する
- USB シリアルからのパラメータ設定（本フォークで追加、2026-10-22）：`paramSerialPoll()`がシリアル入力を1行ずつ読み、次のコマンドを処理する。入力が来るまで待たずにすぐ戻るので、モード選択（§3）と調整メニュー（§5）の待機ループから呼んでいる。走行中や調整項目の実行中は受け付けない
  - `list`：全パラメータを「キー 値」の形式で表示する（`paramPrint()`）
  - `list_r` / `list_a`：走行モード一覧（`MISC::modeListPrint()`）／調整モード一覧（`ADJUST::modeListPrint()`）を表示する。一覧は手書きの表なので、モード構成を変えたら`modeExec()`と合わせて更新する
  - `set <key> <value>`：`paramSet()`→`paramCommit()`で、Web の保存ボタンと同じくすぐ反映・保存する。設定後の値を表示してブザーを1回鳴らすので、範囲外で初期値に戻ったかどうかも表示で分かる。未知のキーを指定するとエラーを表示する
  - その他の入力にはコマンドの一覧を表示する。行末は CR・LF のどちらでもよい。1行は64文字まで
- `parameter.h` の値とは別枠。**黄スイッチ押しながら起動すると `flashInit()` で `parameter.h` 初期値へ戻る**（Web で変更した値は消える）
- 電源を切っても残るので、帰路でスタックしてリセットしても保存済み情報で最短走行を再開できる（モード2がゴール到達時とスタート帰還時に保存する理由）。起動時に`mapCopy()`で読み込むので、探索も前回の地図から続く（[スラローム機能_実装まとめ.md §2.61](スラローム機能_実装まとめ.md)）
- **注意（本フォークで判明、2026-09-23）**：新しいファームウェアで`parameter.h`にパラメータを追加しても、`flashInit()`（黄スイッチ起動）していない機体の`/parameters.txt`にはそのキーの行が無い。`paramRead()`はファイルに存在する行しか反映しないため、読み込みループ**前**にそのパラメータへ`*_INIT`のデフォルトを代入しておかないと0のまま放置される（`th_sen_fr/fl_approach`で実際に発生した不具合、詳細は[スラローム機能_実装まとめ.md §2.19](スラローム機能_実装まとめ.md)）。今後パラメータを追加する際はこのデフォルト代入を忘れないこと

---

## 10. Web サーバ（`webserver.cpp`）

- 実プロジェクトは `#define AP_MODE` 有効。SSID `PICO4_SRDC` / パスワード `12345678` のアクセスポイントになり `http://192.168.4.1`
- ブラウザ上で バッテリ電圧・センサ値（`SIDE VALUE` / `FRONT VALUE`）を表示し、`SIDE REF`（目標値）、`SIDE/FRONT Threshold`（しきい値）、`Gain`、`TIRE_DIAMETER`、`TREAD_WIDTH`、迷路の大きさ `MAZE SIZE`、ゴール座標 `AXIS X/Y`、加速度・速度などを変更 → Enter か Save で `paramWrite()` により Flash 保存、`pulse` も再計算
- 本フォークで追加：入力欄の`name`は`/parameters.txt`のキーと同じにしてあり、保存（`/get`）では表`param_table`を順に回して`paramSet()`で範囲チェックしながら設定し、`paramCommit()`（§9）で反映・保存する（[スラローム機能_実装まとめ.md §2.65](スラローム機能_実装まとめ.md)）
  - `Sensor Parameter`：`Sensor ratio`[%]と、REF・しきい値のベース値。括弧内に倍率を掛けた実際の値を表示する（[スラローム機能_実装まとめ.md §2.63](スラローム機能_実装まとめ.md)）。`FRONT measured(wall / no wall)`行には調整モード10の測定値を表示する（入力欄ではない、[スラローム機能_実装まとめ.md §2.67](スラローム機能_実装まとめ.md)）
  - `Slalom Parameter`：`Slalom speed`・`Slalom inner ratio`・`Slalom outer gain`・`Slalom ramp len`と、計算した`Slalom offset`（[スラローム機能_実装まとめ.md §2.65](スラローム機能_実装まとめ.md)）
  - `Preset`：プリセット1〜3の速度%・加速度%（[スラローム機能_実装まとめ.md §2.64](スラローム機能_実装まとめ.md)）
- 本フォークで追加：`SIDE/FRONT Threshold`の表に`FRONT Approach Threshold(FL+FR)`(`th_f_appr_sum`、`g_sensor.th_approach_sum`)を追加。モード8`adachiDynamic()`の`oneStepDynamic()`が使う走行中到達判定しきい値で、FR+FLの和と比較する（当初はFR/FL個別の2欄、2026-09-25に和の1欄へ変更。詳細は[スラローム機能_実装まとめ.md §2.18・§2.46](スラローム機能_実装まとめ.md)）。`Flash.cpp`に`th_sen_f_approach_sum`キーとして配線済み（妥当性チェックなし、既存の`th_sen_fr`/`th_sen_fl`と同じ扱い）
- 本フォークで追加：同じ表に`FRONT Approach Threshold(Slalom, FL+FR)`(`th_f_appr_sum_sl`、`g_sensor.th_approach_sum_slalom`)を追加。モード9`adachiSlalomDynamic()`専用の到達判定しきい値（和）（モード8用は追い足し前提の値のため流用不可、詳細は[スラローム機能_実装まとめ.md §2.26・§2.46](スラローム機能_実装まとめ.md)）。`th_sen_f_approach_sum_slalom`キーとして同じ扱いで`Flash.cpp`に配線済み
- 本フォークで追加：`SIDE/FRONT Threshold`の表に`SIDE Threshold(Slalom)`(`th_left_sl`/`th_right_sl`、`g_sensor.sen_l/r.th_wall_slalom`)を追加。モード9専用の側方壁判定しきい値（既存`th_wall`はモード9の早め停止位置では誤判定するため新設、詳細は[スラローム機能_実装まとめ.md §2.28](スラローム機能_実装まとめ.md)）。`th_sen_r/l_slalom`キーとして`Flash.cpp`に配線済み。初期値`th_wall`と同値（68/75）のまま未変更で運用中（誤Uターンの解消には`uTurn()`旋回方向切り替え(§2.29)が主に寄与、§2.30参照）
- 非同期 WebServer のためセンサ値がばらつく（大きい値が出ることがあり、小さい値が正しい）。
  §7 のフィルタ（メディアン3＋IIR）は `sen_*.value` に入るので webserver にも効くが、WiFi/AsyncTCP がセンサ割り込みと `analogRead` を突発的に妨げ、フィルタを抜ける**上向きスパイクが残る**（adcView は WiFi OFF なので目立たない）。
- フォークで追加：webserver 表示専用の安定化レイヤ（`webserver.cpp` `webSenStable()`）。`/left_value` `/right_value` `/left_front_value` `/right_front_value` の各エンドポイントで、HTTP リクエストごとに `.value` を 5 個リングに貯めて**メディアン**を返す（200 ms ポーリング → ≒1 秒窓）。`sensor.cpp` の `senMedian3` と同じ「数発の外れ値を捨てる」考え方の表示専用版。判定・制御に使う `sen_*.value` 自体は変更しない。
- 本フォークで整理：トップページの HTML 生成を、`html += "..."` の羅列から `htmlPageHead()` / `htmlBatterySection()` など**セクションごとのヘルパー関数**に分割（`webserver.cpp`）。値を埋め込まない静的な部分（CSS・パラメータ説明文・クライアント側 JS）は生文字列リテラル（`R"HTML(...)HTML"`）にまとめた。出力される HTML・エンドポイント・入力欄名・JS の挙動は整理前と同じ。
- 本フォークで追加：`htmlModeSection()` を追加し、トップページに走行モード一覧（§4）・調整モード一覧（§5）の番号と挙動の早見表を表示するようにした（2026-09-24）。パラメータ調整時に「モードN」がどれを指すか都度このドキュメントを参照しなくて済むようにする目的。内容は静的で、モード構成を変えたときはここも合わせて更新が必要（USB シリアルの`list_r`/`list_a`の表も同様、§9）
- ルータ接続する場合は `AP_MODE` をコメントアウトし SSID/パスワードを書き換え、割当 IP をシリアルモニタで確認（ツール → USB CDC On Boot を Enable にしてビルド）

---

## 11. 迷路完走のための調整手順（マニュアル p.92〜）

1. 壁センサの物理調整：横センサは壁に対し 15〜45°外向き、前センサは 1〜5°外向き、光が壁の高さを越えないよう上下も調整、左右対称に
2. 基準値・しきい値（Web またはモード1）
   - `REF_SEN_R/L`：区画中央での横センサ値
   - `TH_SEN_R/L`：反対の壁に密着させた時の値（有無で 100 以上差が必要）
   - `TH_SEN_FR/FL`：調整モード10で測る。区画中央に置き、1区画先（次の区画の奥）に壁がある場合とない場合の平均をしきい値にする。マニュアルの「柱から 3〜4 cm 手前で計測」とは異なる。壁の判定は区画境界で次の区画の前壁を見て行うので、同じ区画の前壁（約90mm先）で測ると高すぎて前壁を見落とす（[スラローム機能_実装まとめ.md §2.67](スラローム機能_実装まとめ.md)）
3. ゲイン調整（モード4、前壁まで直進）：既定 0.3。大きいほど速く収束するが振動・脱調のリスク（本フォークでモード2→4に変更）
4. タイヤ直径（モード5、3 区画直進）：`TIRE_DIAMETER_INIT`（現在49.5mm）初期。`x = 現径 × 実走行距離 / 目標距離` で補正、誤差 1 mm 以下まで反復（番号は変更なし）
5. トレッド（モード2、720°旋回）：`TREAD_WIDTH_INIT`（現在67.5mm）初期。`x = 目標角 / 実旋回角 × 現トレッド` で補正（本フォークでモード3→2に変更）
6. スラローム調整（モード3、4x4コース最外周を直進→`slalomCell()`で周回、本フォークで追加）：旋回が区画に収まるか、つなぎで脱調しないかを確認し、`Slalom inner ratio`・`Slalom ramp len`・`Slalom outer gain`を調整する。Web UI の`Slalom offset`が90mmを超えないようにする。単発の旋回は調整モード9で確認できる
7. 連続旋回・U字の通し確認（モード7、本フォークで新規追加）：`slalomSequenceCheck()`でS字・旋回直後の直進・U字を一続きに走らせる
8. ゴール座標：ゴールは長方形の対角2座標（Web の `AXIS` と `AXIS2`）で指定する。STEP8 の `parameter.h` は練習用の (3,3)-(3,3)（1区画）。大会は16x16で中央4区画 (7,7)-(8,8)（本番用の STEP9 の初期値。詳細は[スラローム機能_実装まとめ.md §2.56](スラローム機能_実装まとめ.md)）

### 各パラメータの効果（目安）

**本フォークで追加**：Web UI（`webserver.cpp` トップページ最下部）にも同内容を掲載。上げすぎ・下げすぎたときに機体がどう挙動するかの目安。

| パラメータ | 大きくすると | 小さくすると |
| --- | --- | --- |
| `Gain`（`con_wall.kp`） | 壁センサとの距離ずれの補正が強くなり中央へ速く収束するが、大きすぎると蛇行・振動・脱調する | 補正が弱く、壁に寄ったままなかなか中央に戻らない |
| `SIDE REF`（`REF_SEN_R/L`） | これがずれていると`Gain`を合わせても中央からずれた位置に収束してしまう（大小ではなくズレそのものが問題） | 同左 |
| `SIDE/FRONT Threshold`（`TH_SEN_*`） | 実際は壁があるのに「壁なし」と誤判定する（見落とし） | 壁が無いのに「壁あり」と誤判定し、不要に曲がる/止まる |
| `TIRE_DIAMETER` | 1パルスあたりの距離を大きく見積もり、目標距離の手前で止まる（アンダーラン気味） | 逆に目標距離を通り過ぎる（オーバーラン気味） |
| `TREAD_WIDTH` | 回転量を過小に見積もり、90°回りきる前に旋回を止めてしまう（曲がり不足） | 曲がりすぎる（オーバーターン） |
| `Search accel`（直進加速度） | 短い区間でも最高速に届きやすいが、急すぎるとスリップ・脱調しやすい | 滑らかだが、短い区間では目標速度に届く前に距離を使い切り実効速度が落ちる |
| `Turn accel`（`rotate()`の旋回加速度） | 短時間で回れるが、急だと機体が振られる/脱調しやすい | ゆっくり確実だが旋回に時間がかかる |
| `Slalom inner ratio`（旋回中央での内輪の速度比） | 大回りになり張り出しが大きくなる | 小回りになる。0でほぼ内輪を軸にした旋回 |
| `Slalom ramp len`（内輪を落とす/戻す間の外輪の走行距離） | 滑らかに曲がり始めるが張り出しが大きくなる | 張り出しは小さいが急に曲がり始め、姿勢が乱れやすい |
| `Slalom outer gain`（旋回中央での外輪の増速率） | 同じ小回りのまま旋回が速く終わるが、外輪が速くなりスリップしやすい | 外輪は増速しない（0） |
| `Slalom speed`（スラローム旋回中の並進速度） | 旋回が速く終わるが、遠心力でスリップしやすい（軌跡は変わらない） | 確実だが旋回に時間がかかる |
| `Sensor ratio`（センサのベース値に掛ける倍率） | しきい値・REFがまとめて大きくなる（センサ値が練習場より高い会場用） | まとめて小さくなる |
| `Search speed`（探索速度） | 探索は速いが、壁センサの読み取りや姿勢制御が追いつかず衝突・誤判定のリスクが増える | 安定するが探索に時間がかかる |
| `max speed`（最短走行の直進最高速） | タイムは縮むが、加速に必要な距離やカーブ手前での減速が間に合うかの余裕が減る | 安全だが記録が伸びない |

### `parameter.h` 初期値

| マクロ | 値 | 意味 |
| --- | --- | --- |
| `TIRE_DIAMETER_INIT` | 49.5 | タイヤ直径 [mm]（実車調整済み、当初48.21→47.5→49.5） |
| `TREAD_WIDTH_INIT` | 67.0 | トレッド [mm]（実車調整済み、当初65.4→68.3→66.1→67.5→67.0） |
| `REF_SEN_R_INIT` / `REF_SEN_L_INIT` | 338 / 261 | 横センサ目標値（実車再調整済み、当初409/585） |
| `TH_SEN_R_INIT` / `TH_SEN_L_INIT` | 116 / 89 | 横センサ壁有無しきい値（実車再調整済み、当初120/120） |
| `TH_SEN_FR_INIT` / `TH_SEN_FL_INIT` | 32 / 20 | 前センサ壁有無しきい値。FR+FLの和で判定する（調整モード10で測定、当初99/67→30/15→28/7→38/52。[スラローム機能_実装まとめ.md §2.67](スラローム機能_実装まとめ.md)） |
| `SEN_FR_WALL_INIT` / `SEN_FL_WALL_INIT` / `SEN_FR_NOWALL_INIT` / `SEN_FL_NOWALL_INIT` | 54 / 37 / 10 / 3 | 調整モード10の測定値（区画中央、1区画先の前壁あり/なし）。記録用で判定には使わない（[スラローム機能_実装まとめ.md §2.67](スラローム機能_実装まとめ.md)） |
| `TH_SEN_F_APPROACH_SUM_INIT` | 290 | `oneStepDynamic()`(モード8)の走行中到達判定しきい値。FR+FLの和と比較する（本フォークで追加。FR/FL個別の`TH_SEN_FR/FL_APPROACH_INIT`(最終112/160)から2026-09-25に置き換え、初期値290から実車調整で280→290。詳細は[スラローム機能_実装まとめ.md §2.18〜§2.24・§2.46・§2.55](スラローム機能_実装まとめ.md)） |
| `TH_SEN_F_APPROACH_SUM_SLALOM_INIT` | 140 | `adachiSlalomDynamic()`(モード9)専用の走行中到達判定しきい値（和）。§2.65でモード8と同じ値にした後、実車調整値の140を反映（§2.67） |
| `TH_SEN_R_SLALOM_INIT` / `TH_SEN_L_SLALOM_INIT` | `TH_SEN_R/L_INIT`（116 / 89） | `adachiSlalomDynamic()`(モード9)専用の側方壁判定しきい値（本フォークで追加。既存`th_wall`は真の区画中心での停止を前提とした校正値で、モード9の早め停止位置では誤判定するため新設。`th_wall`と同値で運用しているため`TH_SEN_R/L_INIT`を参照する。詳細は[スラローム機能_実装まとめ.md §2.28・§2.30・§2.55](スラローム機能_実装まとめ.md)） |
| `GOAL_X_INIT` / `GOAL_Y_INIT` / `GOAL_X2_INIT` / `GOAL_Y2_INIT` | 3 / 3 / 3 / 3 | ゴール区画（長方形の対角2座標、§2.56）。STEP9は 7 / 7 / 8 / 8 |
| `CON_WALL_KP_INIT` | 0.3 | 姿勢制御 P ゲイン |
| `SEARCH_ACCEL_INIT` | 1.5 | 直進加速度 |
| `TURN_ACCEL_INIT` | 0.4 | `rotate()`専用の旋回加速度 |
| `SLALOM_SPEED_INIT` | 400 | `slalom()`の旋回中の並進速度（外輪の基準速度）。§2.65以降は旋回の軌跡に影響しない |
| `SLALOM_INNER_RATIO_INIT` / `SLALOM_OUTER_GAIN_INIT` / `SLALOM_RAMP_LEN_INIT` | 0.1 / 0.0 / 30.0 | スラロームの形状（内輪の最低速度比、外輪の増速率、内輪を落とす/戻す間の外輪の走行距離[mm]）。張り出しは約57mm（[スラローム機能_実装まとめ.md §2.65](スラローム機能_実装まとめ.md)） |
| `SEN_RATIO_INIT` | 100 | センサのベース値に掛ける倍率[%]（[スラローム機能_実装まとめ.md §2.63](スラローム機能_実装まとめ.md)） |
| `PRESET_SPEED_PCT_INIT` / `PRESET_ACCEL_PCT_INIT` | {100, 90, 110} | 速度・加速度プリセット[%]（[スラローム機能_実装まとめ.md §2.64](スラローム機能_実装まとめ.md)） |
| `UTURN_BACK_LEN_INIT` / `WALL_RETURN_LEN_INIT` / `UTURN_SPEED_INIT` | 40 / 25 / 100 | `uTurn()`と`alignStartPosition()`のバック距離[mm]・前進距離[mm]・速度[mm/s]（本フォークで追加。距離は当初バック・前進で兼用し20→25→30→25で実車調整、2026-09-25に分離してバックを40へ。速度は未調整。詳細は[スラローム機能_実装まとめ.md §2.49](スラローム機能_実装まとめ.md)。前進距離は`UTURN_FWD_LEN_INIT`から改名、[スラローム機能_実装まとめ.md §2.69](スラローム機能_実装まとめ.md)） |
| `SEARCH_SPEED_INIT` | 600 | 探索速度 |
| `MAX_SPEED_INIT` | 800 | 最高速度（最短走行の直進） |
| `MIN_SPEED` | 30 | 最低速度 |
| `HALF_SECTION` / `SECTION` | 90 / 180 | 半区画 / 1区画 [mm] |

---

## 12. マニュアルと実プロジェクトの主な差分

| 項目 | マニュアル | 実プロジェクト |
| --- | --- | --- |
| 調整モード割り当て | 2=タイヤ直径, 5・6=なし | 2=トレッド(720°旋回), 3=4x4コース周回確認`circuitCheck()`（本フォークで新規追加）, 4=P ゲイン(前壁まで直進), 5=タイヤ直径(3 区画), 6=`adcView()`（本フォークで追加）, 7=連続旋回・180度旋回通し確認（本フォークで新規追加）, 10=前センサしきい値の校正`frontThresholdCalib()`（本フォークで新規追加）。旧構成にあった「4=迷路情報表示」は`mapView()`関数ごと削除（§2.57） |
| トレッド初期値 | 例では 64 mm | `TREAD_WIDTH_INIT = 67.0`（実車調整済み、§11参照） |
| AP モード SSID | `PICO4`（パスワード `12345678`） | `PICO4_SRDC` |
| ファイル拡張子 | `.ino` 連結 | 各 `.cpp` + `pico_common.h` に分離 |
| ゴール座標 | `map_manager` で 0x07,0x07 | ゴールは長方形の対角2座標。`paramRead()` が `parameter.h` の値（STEP8 は (3,3)-(3,3)、STEP9 は (7,7)-(8,8)）で上書きし、`goalValidate()` で迷路外・スタートを含む指定を初期値へ戻す |
| 走行モード1（左手法）の終了 | 無限ループ（電源/リセットでしか止まらない） | `while (switchGet() == 0)` でスイッチ押下により停止しモード選択へ戻る |
| 走行モード4 | なし | `SEARCH::lefthandGetWall()` を追加：壁情報を記録する左手法。ゴール or スイッチで停止し `mapWrite()` 保存 |
| センサ AD 取得 | タイマ1（4 kHz ISR）で `for(300){nop}` 待ち＋1回読み（生値） | **core1 固定タスク**で 1 ms 周期 4 ch。`delayMicroseconds(32)` 待ち＋点灯時間棄却＋メディアン3＋IIR。`WAITLOOP_SLED`→`SLED_SETTLE_US`/`SEN_IIR_A`/`SLED_WINDOW_MAX_US`。詳細は §7 |
| 旋回動作 | `rotate()`（超信地旋回）のみ | `rotate()`に加え **`slalom()`/`slalomCell()`を追加**：停止せず、外輪の増速と内輪の減速で曲がる（§2.65で方式を変更）。`RUN::interrupt()`に`turn_enable`/`turn_f`ブランチ、`TMC5240::readXactualLR()`、パラメータ`slalom_speed`/`slalom_inner_ratio`/`slalom_outer_gain`/`slalom_ramp_len`を新設。調整モード3（`circuitCheck()`）・7（`slalomSequenceCheck()`）・9（`slalomTurnCheck()`）で確認可能。`search.cpp`（モード6）・`fast.cpp`（モード7）・`search_dynamic.cpp`（モード9）に組み込み済み（詳細は §5・§6） |
| 走行モード5・6・7 | なし | `SEARCH::lefthandSlalom()`（モード5）・`SEARCH::adachiSlalom()`（モード6）・`FAST::runSlalom()`（モード7）を追加（本フォークで追加、詳細は §4）。`adachiSlalom()`/`runSlalom()`は90°旋回を`slalomCell()`に置き換え、行き止まり(`rear`)は`uTurn()`。`lefthandSlalom()`は自己位置・壁情報を管理しないため90°旋回は`rotate()`のまま、行き止まりの`uTurn()`のみ`lefthand()`と異なる |

実プロジェクトは RT 純正サンプルからのフォークで、STEP8 は上記の調整モード再割り当て・スラローム機能追加・ファイル構成変更・Web サーバ拡張などの改変が中心。
