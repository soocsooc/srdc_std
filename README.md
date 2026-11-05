# srdc_std

SRDC2026 標準機体用の Arduino スケッチ集です。STEP1〜7 でハードウェアを 1 つずつ動かし、STEP8 で迷路を探索して最短経路を走ります。

## 機体

| 項目 | 内容 |
| --- | --- |
| マイコン | ESP32-DevKitC-32E（無印 ESP32） |
| モータドライバ | EIC4311（SLA7078 互換）。CLOCK と CW/CCW で駆動する |
| LED | 4 個（GPIO 5, 13, 16, 15）。HIGH で点灯 |
| スイッチ | SW1（GPIO33）、SW2（GPIO35） |
| 壁センサ | 前・右・左の 3 組（前は 1 個） |
| 電池 | LiPo 3 セル |

SW1 が「次」、SW2 が「決定」です。SW2 の GPIO35 は入力専用ピンで内部プルアップがないので、SW2 をプルアップしていないなら各スケッチの `USE_SW2` を 0 にします。SW2 を読まず、SW1 の短押しを「次」、長押し（1 秒以上）を「決定」にします。

ピンは回路図 V5（11/3 版）に合わせてあります。一覧は [srdc_8_micromouse/STEP8_動作まとめ.md §2](srdc_8_micromouse/STEP8_動作まとめ.md) にあります。

## スケッチ

| スケッチ | 内容 |
| --- | --- |
| `srdc_1_LED` | LED を点滅させる |
| `srdc_2_SWITCH` | SW1 で LED0・1、SW2 で LED2・3 を点けたり消したりする |
| `srdc_3_Buzzer` | 「次」でモード番号を進め、「決定」でモードごとの音を鳴らす |
| `srdc_4_Sensor` | 壁センサと電池電圧をシリアルに表示する |
| `srdc_5_Straight` | スイッチを押すと 2 区画直進する |
| `srdc_6_rotate` | スイッチを押すと右 90°・左 90°・右 180°・左 180° と回る |
| `srdc_7_P_control` | 「決定」で横の壁を見ながら 4 区画直進する。「次」でセンサ値を表示する |
| `srdc_8_micromouse` | 迷路を探索して最短経路を走る。使い方は [STEP8_動作まとめ.md](srdc_8_micromouse/STEP8_動作まとめ.md)、設計の理由は [設計メモ.md](srdc_8_micromouse/設計メモ.md) |

STEP5〜7 のピンとタイヤ径などは、各スケッチの `pins_config.h` にまとめてあります。STEP8 では `device.h` と `parameter.h` です。

## ビルド

各スケッチの `sketch.yaml` の `esp32` プロファイルで、ボードとライブラリの版を固定しています。

- Arduino-ESP32 3.3.11（FQBN `esp32:esp32:esp32`）
- Async TCP 3.5.0、ESP Async WebServer 3.12.0（STEP8 で使う）

```bash
arduino-cli compile --profile esp32 srdc_8_micromouse
arduino-cli upload -p COM3 --profile esp32 srdc_8_micromouse
```

`.ino` と `.h` はリポジトリの [.clang-format](.clang-format) で整形します。CI はスケッチのコンパイルと、clang-format による整形のチェックを行います。

VS Code の clangd は `.vscode/compile_commands.json` を使ってヘッダを解決します。このファイルは Git で管理していないので、クローンした後やスケッチのファイルを増やした後に `python tools/make_compile_db.py` で作り直します。

## 実機で確認・調整が必要な値

次の値はまだ実機で確かめていない仮の値です。

| 項目 | 場所 | 確認方法 |
| --- | --- | --- |
| 1 回転あたりのパルス数（400） | `PULSE_PER_REV` | モータを 1 回転させる |
| モータの回転方向 | `CW_R_FORWARD`／`CW_L_FORWARD` | STEP5 で前進しなければ反転する |
| 受光素子の割り当て（前 36・右 39・左 34） | `AD_F`／`AD_R`／`AD_L` | STEP4 でセンサ値を表示する |
| モータの電流 | 基板のトリマ R26 | 脱調や発熱を見ながら合わせる |
| タイヤ径・トレッド幅・センサの基準値としきい値 | `parameter.h`（STEP8）、`pins_config.h`（STEP5〜7） | STEP8 の調整メニューと Web 調整で校正する |

STEP8 の調整の順序は [STEP8_動作まとめ.md §11](srdc_8_micromouse/STEP8_動作まとめ.md) にあります。

## PC シミュレーター

[tools/step8_sim](tools/step8_sim/README.md) で、STEP8 の迷路ロジック（探索と最短走行）を PC 上でランダムな迷路に対して確かめられます。

## License

(C) 2025 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。明記されていない場合は、Apache License, Version 2.0 に基づき公開されています。ライセンスの全文は [LICENSE](./LICENSE) にあります。
