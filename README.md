# srdc_std

SRDC2026標準機体（ESP32-DevKitC-32E、モータドライバ EIC4311）用の Arduino スケッチ集です。
[soocsooc/micromouse](https://github.com/soocsooc/micromouse)（Pi:Co Classic4 用）の STEP1〜8 を移植しました。
モータの駆動は Pi:Co Classic3 のサンプルと同じ方式で、タイマー割り込みでステップパルスを出します。

## 動作環境

- Arduino CLI（各スケッチの `sketch.yaml` の `esp32` プロファイルで版を固定）
- Arduino-ESP32 3.3.11（FQBN `esp32:esp32:esp32`）
- Async TCP 3.5.0、ESP Async WebServer 3.12.0

```bash
arduino-cli compile --profile esp32 srdc_8_micromouse
arduino-cli upload -p COM3 --profile esp32 srdc_8_micromouse
```

## Pi:Co Classic4 版との違い

| 項目 | 標準機体 |
| --- | --- |
| LED | GPIO 5, 13, 15, 16。**LOW で点灯** |
| スイッチ | SW1 = GPIO33（次へ）、SW2 = GPIO35（決定）。−1 の操作はない。起動時に SW2 を押すとフラッシュを初期化する |
| SW2 のプルアップ | GPIO35 は内部プルアップがないため、**SW2 と 3.3V の間に 10k の外付けが必要** |
| 壁センサ | 前・右・左の 3 個（前は 1 個）。STEP8 は前の値を FR と FL の両方に入れて、上位の処理を Classic4 版のまま使う |
| モータ | MOTOR_EN = GPIO17（**HIGH で有効**）、CLOCK R/L = 19/18、CW/CCW R/L = 25/26 |
| 残量 LED | なし。電池電圧が下がるとブザーで知らせる（LiPo 3 セル、10.0V 未満） |
| Web 調整の AP | SSID `SRDC_STD`（192.168.4.1） |

ピン定義は STEP8 では `srdc_8_micromouse/device.h`、STEP5〜7 では各スケッチの `pins_config.h` にまとめてあります。

## 実機で確認・調整が必要な値

標準機体ではまだ校正していないため、次の値は仮置きです。

- **1 回転あたりのパルス数** `PULSE_PER_REV`（400）：M2=H、M1/M3=L の前提。1 回転させて確かめる
- **モータの回転方向** `CW_R_FORWARD` / `CW_L_FORWARD`：前進しなければ反転する
- **受光素子の割り当て** `AD_F`/`AD_R`/`AD_L`（GPIO 36/39/34）：センサ値を表示して確かめる
- **タイヤ径・トレッド幅・センサのしきい値**：Pi:Co Classic3 の値が入っている。STEP8 の調整メニュー（モード 15）と Web 調整で校正する
- **モータ電流**：基板のトリマ R26 で合わせる

## PC シミュレーター

`tools/step8_sim` で STEP8 の迷路ロジックを PC 上で確かめられます。使い方は `tools/step8_sim/README.md` を参照してください。

## 資料

`srdc_8_micromouse/STEP8_動作まとめ.md` と `スラローム機能_実装まとめ.md` は、Classic4 版の資料をそのまま持ってきたものです。コード中の `§` 番号はこの資料を指します。TMC5240 に関する記述は標準機体には当てはまりません。

## License

元のスケッチは (C) 2025 RT Corporation で、Apache License, Version 2.0 に基づき公開されています。詳しくは [LICENSE](./LICENSE) を参照してください。
