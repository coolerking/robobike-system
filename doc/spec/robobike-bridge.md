# robobike_bridge 仕様書

| 項目 | 内容 |
| --- | --- |
| パッケージ | `src/rpi5/robobike_bridge`（ament_python）、メッセージは `src/common/robobike_msgs` |
| 実行環境 | Raspberry Pi 5 / `docker/rpi5` コンテナ（ROS 2 Humble、`network_mode: host`） |
| 対象ファームウェア | ROBOBIKE（ESP32-C3、ESP-IDF）[tneurjpy-cloud/robobike](https://github.com/tneurjpy-cloud/robobike) `PROGVER 1042`（2026-09-27）で確認 |
| フェーズ1の範囲 | ROBOBIKE のテレメトリを**読み取り専用**で取得し、ROS 2 トピックとして publish する（ROBOBIKE への操作指令は送らない） |

## 1. 背景と全体構成

ROBOBIKE は ROS 非対応の自転車ロボットで、自身が Wi-Fi AP（SSID `ROBOBIKE-XXXXXX`、パスワードなし、`192.168.4.1`）兼 HTTP サーバとして動作する。
ファームウェアは `/` に最初にアクセスしたクライアントの IP を **MASTER**（操縦画面 `root.html`）として記憶し、それ以外のクライアントには**モニター画面**（`monitor.html`）を返す。
モニター画面は `GET /get_acc` を 50 ms 周期でポーリングし、制御ループ（250 Hz）が記録したサンプルを CSV で受け取って描画している。
`robobike_bridge` はこのモニター画面と同じ API を使い、2 台目のクライアントとしてテレメトリを取得する。

```text
                       Wi-Fi AP 192.168.4.0/24 (ROBOBIKE, max 2 clients)
  [スマホ: MASTER] ───────┐
   操縦/ニュートラル調整    │          ┌──────────── Raspberry Pi 5 ─────────────┐
                         ├── wlan0 ──┤ docker rpi5 (host network)              │
  [ROBOBIKE ESP32-C3] ────┘          │  pi5_camera      → /camera/image_raw     │
   http://192.168.4.1                │  robobike_bridge → /robobike/telemetry   │
                                     └──────────── eth0 ─────────────────────────┘
                                                    │ 有線LAN（インターネット、apt、DDS）
                                              [推論/記録 PC] robobike_policy, robobike_logger
```

## 2. 運用手順（前提条件）

1. ROBOBIKE の電源を入れ、**1 台目としてスマホ**を AP に接続する（スマホが MASTER になる）。
2. スマホで（初回のみ）ニュートラル調整を行い、SAVE/RETURN で操縦画面に遷移させる。
3. Raspberry Pi 5 は eth0 で有線 LAN（インターネット・推論 PC）に接続済みとする。wlan0 を手動で ROBOBIKE の AP に **2 台目**として接続する（§6 のネットワーク設定が必須）。
4. `docker compose -f docker/rpi5/docker-compose.yml up -d --build` でコンテナを起動する。
5. `docker compose ... exec rpi5 bash` → `ros2 run pi5_camera camera_node` でカメラ画像を publish する。
6. 別の `docker compose ... exec rpi5 bash` → `ros2 run robobike_bridge bridge_node` でブリッジを起動する。
7. ブリッジは §4 のトピックを継続的に publish する。

制約（ファームウェア仕様に由来）:

- AP の同時接続数は **2**（`max_connection = 2`）。スマホと Pi 以外の端末は接続できない。
- MASTER は「`/` に最初にアクセスした IPv4」で、ROBOBIKE を再起動するまで変わらない。**Pi が先に `/` へアクセスすると Pi が MASTER になり、スマホで操縦できなくなる**。そのためブリッジは `/` に一切アクセスしない（§3.3）。Pi のブラウザで ROBOBIKE を開くことや、OS のキャプティブポータル検出で `/` が自動的に開かれることにも注意する。
- オートスリープは `/command` の受信時刻（`userLastControlTime`）で判定される。`/get_acc` はこの時刻を更新しないため、ブリッジがあっても停車中に操作が 15 分間ないと ROBOBIKE はスリープする（スマホの操縦画面は `only_data` コマンドを定期送信するため、開いている間はスリープしない）。

## 3. ROBOBIKE 側インターフェース（調査結果）

### 3.1 `GET http://192.168.4.1/get_acc`

- 応答: `Content-Type: text/csv; charset=UTF-8`、1 サンプル 1 行（`\n` 区切り）、**ヘッダ行なし**。
- 動作: ファームウェア内のリングバッファ（`RING_BUF_SIZE = 250 Hz × 5 s = 1250` サンプル）から**前回の読み出し以降の未読サンプル**を返し、読み出し位置（`index_r`）を進める。応答サイズは最大 64 KiB（`CTL_DATA_BUFSIZE`、約 600 行）。
  - 読み出し位置はファームウェア全体で 1 つしかない。**`/get_acc` を呼ぶクライアントが複数あると、サンプルを奪い合う**。ブリッジ稼働中は他端末でモニター画面を開かないこと。
  - 5 秒以上ポーリングが止まると、古いサンプルから上書きされて欠落する。
- サンプル生成周期: 制御タスク `ControlTask` が 1 周期ごとに `put_control_data()` を呼ぶ。`SV_FRQ = 250 Hz`、つまり **4 ms ごと**に 1 サンプル。

### 3.2 CSV フォーマット

現行ファームウェア（v1033 以降、`PROGVER 1042` で確認）は 13 列を出力する。

| # | 列名 | 型 | 単位・意味（ソース上の定義） |
| --- | --- | --- | --- |
| 0 | `HEADER` | char | レコード種別。モニター用は常に `a`（`/command` 応答の設定値レコードは `b`） |
| 1 | `TIME_MS` | uint32 | ROBOBIKE の起動からの経過ミリ秒 `millis()`。約 49.7 日でラップ |
| 2 | `SV_DRV` | float | 駆動サーボ出力 `mot_out` [%]、-100〜+100（後進は負） |
| 3 | `SV_STR` | float | ステアリングサーボ出力 `str_out` [deg]、+ は右 |
| 4 | `SV_STD` | float | サイドスタンドサーボ出力 `ex1_out` [deg] |
| 5 | `SWP_SG` | float | 周波数スイープ用の正弦波信号（SWEEP 時以外は 0） |
| 6 | `SV_POS` | float | ステアリングサーボの実角度（ADC 値を ±90° に換算） [deg] |
| 7 | `GY_ROLL` | float | ロール角速度（オフセット補正済み） [deg/s] |
| 8 | `GY_YAW` | float | ヨー角速度（オフセット補正済み） [deg/s] |
| 9 | `GY_PITCH` | float | ピッチ角速度（オフセット補正済み） [deg/s] |
| 10 | `ACC_X` | float | 加速度 前方 [m/s²] |
| 11 | `ACC_Y` | float | 加速度 右方 [m/s²] |
| 12 | `ACC_Z` | float | 加速度 下方 [m/s²] |

例（現行）: `a,77904,0.000,0.000,36.000,0.000,4.087,-0.023,-0.058,-0.375,2.219,9.339,2.188`

v1032 以前のファームウェアは、要求項目と同じ 8 列（`HEADER,TIME_MS,SV_DRV,SV_STR,SV_STD,GY_ROLL,GY_YAW,GY_PITCH`）を出力する（例: `a,175715,47.000,24.435,80.000,-1.340,27.215,0.095`）。
ブリッジは**列数（8 または 13）で形式を判定**し、どちらにも対応する。

### 3.3 ブリッジが使う／使わないエンドポイント

| エンドポイント | 使用 | 理由 |
| --- | --- | --- |
| `GET /get_acc` | 使う | テレメトリの取得 |
| `GET /clear_buffer` | 起動時のみ使う（パラメータで無効化可） | 起動前に溜まった古いサンプルを捨てる（モニター画面も読み込み時に同じ処理をしている） |
| `GET /` | **使わない** | アクセスすると MASTER 登録が起きる（§2） |
| `GET /command?button=N` | **使わない**（フェーズ1） | 操縦・設定を変更し、スリープ判定にも影響する。ファームウェアは `/command` で MASTER を検証していないので、送信すれば実際に車体が動く |

## 4. ROS インターフェース

### 4.1 メッセージ `robobike_msgs/msg/RobobikeTelemetry`（新規）

```text
# One control-loop sample (250 Hz) read from ROBOBIKE GET /get_acc.
std_msgs/Header header   # stamp: TIME_MS を ROS 時刻に換算した値（§5.3）、frame_id: パラメータ frame_id
string record_type       # HEADER 列（通常 "a"）
uint32 time_ms           # TIME_MS（ROBOBIKE 起動からの経過 ms、生値）
float32 sv_drv           # SV_DRV  [%]
float32 sv_str           # SV_STR  [deg]
float32 sv_std           # SV_STD  [deg]
float32 gy_roll          # GY_ROLL [deg/s]
float32 gy_yaw           # GY_YAW  [deg/s]
float32 gy_pitch         # GY_PITCH[deg/s]
# 以下は v1033 以降の 13 列形式でのみ有効（8 列形式では NaN）
float32 swp_sg
float32 sv_pos           # [deg]
float32 acc_x            # [m/s^2]
float32 acc_y            # [m/s^2]
float32 acc_z            # [m/s^2]
```

- `robobike_msgs/CMakeLists.txt` に `rosidl_generate_interfaces(${PROJECT_NAME} "msg/RobobikeTelemetry.msg" DEPENDENCIES std_msgs)` を追加し、`package.xml` に `std_msgs` の依存を追加する。
- 要求された 8 項目（HEADER, TIME_MS, SV_DRV, SV_STR, SV_STD, GY_ROLL, GY_YAW, GY_PITCH）は必須フィールドとする。単位はファームウェアの値をそのまま使い、変換しない（モニター画面の CSV 保存データと比較しやすくするため）。

### 4.2 Publish トピック

| トピック | 型 | QoS | 周期 | 内容 |
| --- | --- | --- | --- | --- |
| `/robobike/telemetry` | `robobike_msgs/msg/RobobikeTelemetry` | `qos_profile_sensor_data`（BEST_EFFORT、depth 5）。パラメータで RELIABLE / depth 変更可 | **1 サンプル 1 メッセージ（約 250 Hz）**。ポーリング（既定 20 Hz）ごとに、届いた全サンプルを時刻順にまとめて publish | テレメトリ本体 |
| `/robobike/bridge/connected` | `std_msgs/msg/Bool` | RELIABLE + TRANSIENT_LOCAL、depth 1 | 状態が変わったとき | ROBOBIKE からテレメトリを取得できているか |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 既定 | 1 Hz | `robobike_bridge: telemetry` の状態（OK / WARN / ERROR）、受信レート、欠落数、HTTP エラー数、検出した CSV 形式（8 列 / 13 列）、最終受信からの経過時間 |

### 4.3 Subscribe トピック

フェーズ1ではなし。既存スケルトンの `/cmd_vel` 購読（何もしない）は**削除**し、操縦指令の送信はフェーズ2で別途仕様化する（§9）。

### 4.4 パラメータ

| 名前 | 型 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `base_url` | string | `http://192.168.4.1` | ROBOBIKE の HTTP ベース URL |
| `poll_period` | double | `0.05` | `/get_acc` のポーリング周期 [s]。モニター画面と同じ値。0 より大きく、5.0（リングバッファ 5 s 分）未満の有限値でなければ `ValueError` |
| `http_timeout` | double | `0.5` | 接続・読み出しタイムアウト [s]。有限かつ正でなければ `ValueError` |
| `clear_buffer_on_start` | bool | `true` | 起動時（および再接続時）に `/clear_buffer` を呼ぶ |
| `frame_id` | string | `robobike` | `header.frame_id` |
| `topic` | string | `/robobike/telemetry` | テレメトリのトピック名 |
| `qos_reliable` | bool | `false` | `true` で RELIABLE（記録の取りこぼしを避けたい場合） |
| `qos_depth` | int | `5` | QoS の depth。1 以上 |
| `stale_timeout` | double | `1.0` | 新しいサンプルが届かない状態がこの秒数続いたら `connected=false`、診断を WARN にする |
| `reconnect_backoff_max` | double | `5.0` | 通信エラー時の再試行間隔の上限 [s]（0.5 s から倍々に延ばす） |

## 5. 内部動作

### 5.1 ポーリング

- `create_timer(poll_period, poll)` で周期実行する。HTTP 通信は ROS のコールバック内で同期的に行う。`http_timeout` は `poll_period` の数倍以内に収まるため、他のコールバックを長く止めることはない。ただし、ノード内にスレッドは作らず、シングルスレッドの executor で動かす。
- `http.client.HTTPConnection` を**使い回す（keep-alive）**。ESP32 側の HTTP サーバは同時ソケット数が 4（`max_open_sockets = 4`、LRU で破棄）なので、20 Hz で毎回新しい接続を作るとソケットを消費してスマホの操作が詰まるおそれがある。例外や応答異常が起きたら接続を閉じて作り直す。
- HTTP ライブラリは Python 標準ライブラリ（`http.client`）のみを使い、追加の依存を増やさない。

### 5.2 パース

- 応答本文を `\n` で分割し、空行は無視する。各行を `,` で分割する。
- 列数が 13 なら現行形式、8 なら旧形式として扱う。それ以外の列数や、`HEADER` が `a` でない行、数値に変換できない行は**その行だけ捨てて**、診断のカウンタ（`malformed`）を増やす。
- `float('nan')`、`inf` もそのまま通す（ファームウェアの値を加工しない）。
- 形式が途中で変わった（OTA 更新など）場合は診断に記録し、新しい形式で処理を続ける。

### 5.3 タイムスタンプ

- `time_ms` には生値を必ず入れる。
- `header.stamp` は ROBOBIKE の時刻を ROS 時刻に換算した値とする。
  - 応答を受信した時刻 `t_recv`（ROS 時刻）と、その応答の最後のサンプルの `time_ms` から `offset = t_recv - time_ms` を求め、**直近 10 秒間の最小値**を時刻オフセットとして使う（ネットワーク遅延を最小限にした推定値）。
  - 各サンプルの `stamp = time_ms + offset`。こうすると、同じ応答に含まれるサンプル間の 4 ms 間隔が保たれる。
- `time_ms` が前回より小さくなった場合（ROBOBIKE の再起動やラップ）は、オフセットの推定をやり直し、診断に記録する。

### 5.4 欠落検出

- 連続するサンプルの `time_ms` の差が `1.5 × (1000 / 250) = 6 ms` を超えたら欠落とみなし、推定欠落数 `round(Δ / 4) - 1` を診断のカウンタに加算する。リングバッファの溢れや通信断が原因になる。
- ROBOBIKE の制御周期 250 Hz は定数としてコードに持つ（パラメータにはしない）。

### 5.5 エラー処理と再接続

| 事象 | 動作 |
| --- | --- |
| 接続失敗・タイムアウト・HTTP 200 以外 | WARN ログ（5 s で throttle）。接続を閉じ、0.5 s から最大 `reconnect_backoff_max` まで倍々に間隔を延ばして再試行する。**ノードは終了しない** |
| 応答は正常だが空（新しいサンプルがない） | 正常として扱う。`stale_timeout` を超えたら `connected=false` にする |
| 再接続に成功した | `clear_buffer_on_start` が true なら `/clear_buffer` を呼んでから再開する。`connected=true` にする |
| 不正な行 | §5.2 のとおり、その行だけ捨てる |
| パラメータ不正 | 起動時に `ValueError` を出して終了する（pi5_camera と同じ方針） |

## 6. ネットワーク設定（Raspberry Pi 5 側の必須事項）

ROBOBIKE の AP にはインターネット接続がない。さらに、ROBOBIKE はキャプティブポータル用の DNS サーバを持ち、**すべての名前解決に 192.168.4.1 を返す**。
wlan0 を既定の設定のまま接続すると、デフォルトルートや DNS が wlan0 側に取られ、apt や推論 PC との通信が壊れるおそれがある。

- wlan0 の接続（NetworkManager の場合）には次の設定をする:
  `nmcli connection modify <ROBOBIKE-xxxx> ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled connection.autoconnect no`
  これで `192.168.4.0/24` だけが wlan0 経由になり、デフォルトルートと DNS は eth0 のまま保たれる。
- ROS 2 の DDS（Fast DDS）は既定で全インターフェースを使う。2 台しか接続できない ROBOBIKE の AP に DDS の multicast を流さないよう、Fast DDS のプロファイル XML（`interfaceWhiteList` に eth0 の IP を指定）で DDS を eth0 だけに限定することを推奨する。`docker/rpi5` の環境変数 `FASTRTPS_DEFAULT_PROFILES_FILE` で指定する（設定ファイルの追加は実装時に行う）。
- コンテナは `network_mode: host` なので、Docker 側の追加設定は不要。

## 7. 起動例

```bash
# rpi5 コンテナ内
source /opt/ros/humble/setup.bash
colcon build --packages-select robobike_msgs pi5_camera robobike_bridge
source /ros2_ws/install/setup.bash
ros2 run robobike_bridge bridge_node
# パラメータ指定の例
ros2 run robobike_bridge bridge_node --ros-args -p poll_period:=0.05 -p qos_reliable:=true

# 確認（PC 側・Pi 側どちらでも）
ros2 topic hz /robobike/telemetry          # 約 250 Hz
ros2 topic echo /robobike/telemetry --once
ros2 topic echo /robobike/bridge/connected
```

`robobike_logger` の `record.launch.py` の記録対象に `/robobike/telemetry` と `/robobike/bridge/connected` を追加する（実装時に一緒に変更する）。

## 8. テスト方針（テスト先行）

`src/rpi5/robobike_bridge/test/` に pytest を作成し、失敗を確認してから実装する。rclpy / `robobike_msgs` が import できない環境では、pi5_camera と同様に stub を `sys.modules` に入れて動かす。

| テスト | 内容 |
| --- | --- |
| パーサ | 13 列・8 列の正常行、空行、列数不正、`HEADER` 不正、数値不正、CRLF。8 列では拡張フィールドが NaN になること |
| パラメータ検証 | `poll_period` / `http_timeout` が 0・負・NaN・inf のとき、および `poll_period >= 5.0` のとき `ValueError` |
| タイムスタンプ | 応答内のサンプル間隔が 4 ms で保たれること。遅延の揺れがあっても最小オフセットが使われること。`time_ms` が巻き戻ったら推定がやり直されること |
| 欠落検出 | `time_ms` の飛びから欠落数が計算されること |
| HTTP 結合 | `http.server` によるフェイク ROBOBIKE（`/get_acc`、`/clear_buffer`）に対して、publish 件数・順序、起動時の `/clear_buffer` 呼び出し、**`/` と `/command` にアクセスしないこと**、タイムアウト・500 応答・切断からの再接続とバックオフ、`connected` の遷移 |
| 実データ | 上流リポジトリ `doc/*.csv` の実データ（8 列形式・13 列形式）を再生して、全行がパースされること |
| 実機（手動） | Pi 5 + ROBOBIKE 実機で、スマホの操縦を妨げないこと（MASTER がスマホのままであること）、`ros2 topic hz` で約 250 Hz、走行中の値が CSV 保存版と一致すること |

## 9. 範囲外・今後の検討事項

- **操縦指令（フェーズ2）**: `/cmd_vel` → `/command?button=...`（`bt_F` 発進、`bt_S` 停止、`bt_Str_S&value=±100` 操舵スライダー、`bt_BK` 後進、`bt_A_Up/Dn` 速度）への対応。ファームウェアは `/command` で MASTER を検証しないため技術的には可能だが、スマホ操作との競合、発進・停止シーケンス中はコマンドが捨てられること、通信が途絶えたときのフェイルセーフがファームウェア側にないこと、の 3 点を安全面から別途設計する必要がある。
- 設定値レコード（`/command` 応答の `b` 行: PROG_VER、ゲイン類）の取得。`/command?button=only_data` は副作用が小さいものの、スリープ判定のタイマーを更新してしまうため、フェーズ1では行わない。
- `sensor_msgs/Imu` 形式での再 publish（軸の向き・符号の定義と rad/s への換算が必要）。
- 複数台の ROBOBIKE（AP が別々）への同時接続。
