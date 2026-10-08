# robobike_bridge 仕様書

| 項目 | 内容 |
| --- | --- |
| パッケージ | `src/rpi5/robobike_bridge`（ament_python）、メッセージは `src/common/robobike_msgs` |
| 実行環境 | Raspberry Pi 5 / `docker/rpi5` コンテナ（ROS 2 Humble、`network_mode: host`） |
| 対象ファームウェア | ROBOBIKE（ESP32-C3、ESP-IDF）[tneurjpy-cloud/robobike](https://github.com/tneurjpy-cloud/robobike) `PROGVER 1042`（2026-09-27）で確認 |
| 機能 | **上り（ROBOBIKE → ROS）**: テレメトリを取得して publish する。**下り（ROS → ROBOBIKE）**: `/cmd_vel`（方策ノード／テレオペを `robobike_control` で調停した結果）を ROBOBIKE の HTTP 操縦コマンドに変換して送信する |

## 1. 背景と全体構成

ROBOBIKE は ROS 非対応の自転車ロボットで、自身が Wi-Fi AP（SSID `ROBOBIKE-XXXXXX`、パスワードなし、`192.168.4.1`）兼 HTTP サーバとして動作する。
ファームウェアは `/` に最初にアクセスしたクライアントの IP を **MASTER** として記憶し、MASTER には操縦画面（`root.html`）を、それ以外のクライアントには**モニター画面**（`monitor.html`）を返す。
ブラウザの操縦画面は、ボタン操作に応じて `GET /command?button=<ID>` を送る。モニター画面は `GET /get_acc` を 50 ms 周期でポーリングし、テレメトリを描画している。

`robobike_bridge` はブラウザの画面を使わず、これらの HTTP API を直接呼び出す。

```text
                       Wi-Fi AP 192.168.4.0/24 (ROBOBIKE, max 2 clients)
  [スマホ: MASTER] ───────┐
   ニュートラル調整・非常停止 │          ┌──────────── Raspberry Pi 5 ──────────────────┐
                         ├── wlan0 ──┤ docker rpi5 (host network)                   │
  [ROBOBIKE ESP32-C3] ────┘          │  pi5_camera      → /camera/image_raw          │
   GET /get_acc   ──上り──────────────▶│  robobike_bridge → /robobike/telemetry        │
   GET /command   ◀─下り───────────────│                  ← /cmd_vel                   │
                                     └──────────── eth0 ──────────────────────────────┘
                                                    │ 有線LAN（インターネット、apt、DDS）
                                [PC] robobike_policy → /policy/cmd_vel ─┐
                                     robobike_teleop → /teleop/cmd_vel ─┤ robobike_control → /cmd_vel
                                     robobike_logger（rosbag）
```

## 2. 2 台目の Pi から操縦できるか（調査結果）

**ファームウェアの実装上は、2 台目（MASTER ではない端末）からも操縦できる**。

- MASTER かどうかを判定しているのは `GET /` のハンドラ（`web_handler.c` の `root_get_handler`）だけで、その用途は「どの HTML を返すか」の切り替えだけである。
- `/command` のハンドラ（`command_handler`）は、送信元 IP も MASTER かどうかも確認しない。クエリの `button` を数値として解釈し、そのまま `put_command()` に渡す。
- 取扱説明書の「2 台目を接続するとモニター画面が表示され、操縦や設定をすることはできません」は、**2 台目に返されるモニター画面に操縦ボタンが無い**という意味である。API が拒否しているわけではない。
- 注意: これはソースコードを読んで得た結論で、実機ではまだ確認していない。実装の最初の段階で、実機により確認する（§9）。

したがって、Pi を 2 台目として接続したまま `robobike_bridge` から操縦できる。スマホは MASTER のまま残り、ニュートラル調整と、**人が行う非常停止（STOP ボタン）**の手段として使う。

## 3. 運用手順（前提条件）

1. ROBOBIKE の電源を入れ、**1 台目としてスマホ**を AP に接続する（スマホが MASTER になる）。
2. スマホで（初回のみ）ニュートラル調整を行い、SAVE/RETURN で操縦画面に遷移させる。**以後、走行中はスマホの操縦画面を開いたまま手元に置き、いつでも STOP を押せるようにする。**
3. Raspberry Pi 5 は eth0 で有線 LAN（インターネット・PC）に接続済みとする。wlan0 を手動で ROBOBIKE の AP に **2 台目**として接続する（§7 のネットワーク設定が必須）。
4. `docker compose -f docker/rpi5/docker-compose.yml up -d --build` でコンテナを起動する。
5. `docker compose ... exec rpi5 bash` → `ros2 run pi5_camera camera_node` でカメラ画像を publish する。
6. 別の `docker compose ... exec rpi5 bash` → `ros2 run robobike_bridge bridge_node` でブリッジを起動する。
   - 既定（`enable_drive:=false`）ではテレメトリの publish だけを行い、ROBOBIKE には操縦コマンドを一切送らない（データ収集・確認用）。
   - ROS から操縦するときは `--ros-args -p enable_drive:=true` を付けて起動する。
7. PC 側で `robobike_control` を起動し（必要に応じて `robobike_teleop`、`robobike_policy` も起動する）、`/control/mode` を `TELEOP` または `AUTO` にすると、`/cmd_vel` に従って ROBOBIKE が動く。

制約（ファームウェア仕様に由来）:

- AP の同時接続数は **2**（`max_connection = 2`）。スマホと Pi 以外の端末は接続できない。
- MASTER は「`/` に最初にアクセスした IPv4」で、ROBOBIKE を再起動するまで変わらない。**Pi が先に `/` へアクセスすると Pi が MASTER になり、スマホで調整・操縦できなくなる**。そのためブリッジは `/` に一切アクセスしない。Pi のブラウザで ROBOBIKE を開くことや、OS のキャプティブポータル検出で `/` が自動的に開かれることにも注意する。
- オートスリープ（停車中に操作が 15 分間ないと deep sleep に入る）は、`/command` を最後に受信した時刻（`userLastControlTime`）で判定される。ブリッジがコマンドを送るとスリープ判定のタイマーがリセットされる。`/get_acc` ではリセットされない。

## 4. ROBOBIKE 側インターフェース（調査結果）

### 4.1 `GET /get_acc`（テレメトリ）

- 応答: `Content-Type: text/csv; charset=UTF-8`、1 サンプル 1 行（`\n` 区切り）、**ヘッダ行なし**。
- 動作: ファームウェア内のリングバッファ（`RING_BUF_SIZE = 250 Hz × 5 s = 1250` サンプル）から**前回の読み出し以降の未読サンプル**を返し、読み出し位置（`index_r`）を進める。応答サイズは最大 64 KiB（`CTL_DATA_BUFSIZE`、約 600 行）。
  - 読み出し位置はファームウェア全体で 1 つしかない。**`/get_acc` を呼ぶクライアントが複数あると、サンプルを奪い合う**。ブリッジ稼働中は他端末でモニター画面を開かないこと。
  - 5 秒以上ポーリングが止まると、古いサンプルから上書きされて欠落する。
- サンプル生成周期: 制御タスク `ControlTask` が 1 周期ごとに `put_control_data()` を呼ぶ。`SV_FRQ = 250 Hz` なので **4 ms ごと**に 1 サンプル。

### 4.2 CSV フォーマット

現行ファームウェア（v1033 以降、`PROGVER 1042` で確認）は 13 列を出力する。

| # | 列名 | 型 | 単位・意味（ソース上の定義） |
| --- | --- | --- | --- |
| 0 | `HEADER` | char | レコード種別。モニター用は常に `a`（`/command` の応答は `b`） |
| 1 | `TIME_MS` | uint32 | ROBOBIKE の起動からの経過ミリ秒 `millis()`。約 49.7 日でラップ |
| 2 | `SV_DRV` | float | 駆動サーボ出力 `mot_out` [%]、-100〜+100（後進は負）。**0 以外なら走行中** |
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

### 4.3 `GET /command?button=<ID>[&value=<N>]`（操縦コマンド）

- `button` は**コマンド名ではなく数値 ID**。ID はファームウェアの `TcmdID` 列挙型（`userdefine.h` の `COMMAND_LIST`、`unknown = 0` から始まる連番）で決まる。
  - `root.html` の `COMMAND_NAMES` には `bt_sweepON` 以降の一部が抜けているため、ID 32 以降はファームウェアと一致しない。ブリッジは ID を**ファームウェアの列挙型に合わせて**定義し、ブリッジが使うのは下表の ID だけとする。
- 応答: 設定値の CSV 1 行（`b,PROG_VER,DATA_VER,STATUS,STR0,MOT_SPD,GAIN_STR,GAIN_W_ROLL,GAIN_DIFF,ANG_STD_NUT,STR_TURN,YAW_COEFF,AUTO_CIRCLING,STR_CMD_RATE,STR_CMD_SPD`）。応答はコマンドが受理されたかどうかに関係なく返る。
- 処理方式:
  - `bt_F` / `bt_S` / `stp_all` などの時間のかかるコマンドは、コマンド処理タスク `cmdProcTask` に非同期で渡され、HTTP の応答はすぐ返る。
  - `cmdProcTask` が実行中（`cmdProc_busy`）の間に届いたコマンドは、`bt_L` / `bt_R` を除いて**黙って捨てられる**（HTTP 応答は通常どおり返る）。`bt_L` / `bt_R` は、実行中のコマンドが終わるまで HTTP サーバのタスクを止めてしまう。
  - ESP32 の HTTP サーバは同時ソケット数が 4（`max_open_sockets = 4`、古いものから LRU で破棄）。

ブリッジが使うコマンド:

| ID | 名前 | ファームウェアでの動作 |
| --- | --- | --- |
| 2 | `only_data` | 何もしない（設定値 CSV の取得のみ）。スリープ判定のタイマーはリセットされる |
| 3 | `bt_F` | **停止中**: スタンドを降ろして車体を起こし、駆動を `MOT_SPD` [%] で開始し、自立制御（`auto_enable`）を有効にしてスタンドを上げる（発進シーケンス、数秒かかる）。**走行中**: 舵を 0 にゆっくり戻すだけ |
| 4 | `bt_S` | **前進中**: 舵を 0 に戻す → 左に舵を切る → スタンドを出す → 自立制御を無効にする → 駆動を停止する（停止シーケンス）。その後、設定値を NVS に保存する（`savenvs()`）。**停止中**: 駆動を 0 にする |
| 7 | `bt_Str_S` + `value=-100..100` | 舵の目標値を `value × STR_TURN / 100` [deg] に設定する（`STR_TURN` の既定値は 40°、+ は右）。自立制御中は、これがバランス制御ループの舵の目標値になる |
| 9 / 10 | `bt_A_Up_0` / `bt_A_Dn_0` | `MOT_SPD` を ±1 する（範囲 0〜60）。走行中なら駆動に即時反映し、停止中は値の変更だけ |
| 8 | `bt_BK` | 停止中なら後進（駆動 -20%、自立制御なし）。後進中なら停止する |
| 1 | `stp_all` | 即時停止: 自立制御を無効にし、駆動・舵・スタンドをニュートラルに戻し、NVS に保存する |

**使わない**コマンド: `bt_A_Up` / `bt_A_Dn`（ID 11 / 12。**停止中でも駆動を開始してしまう**）、`bt_L` / `bt_R`（HTTP サーバを止めることがある）、調整・設定系の ID 13 以降（ゲインやニュートラルを書き換えてしまう）。

ファームウェア側の安全上の性質（ブリッジで補う必要がある点）:

- **通信が途絶えたときのフェイルセーフが無い**。最後に受けたコマンドの状態のまま走り続ける。Wi-Fi の切断（`WIFI_EVENT_AP_STADISCONNECTED`）もログに出すだけで、停止はしない。
- 転倒の判定（舵が `STRMAX - 2` に張り付いた状態が 0.5 s 続く）は行い、その場合は自動で停止する。

### 4.4 ブリッジが使う／使わないエンドポイント

| エンドポイント | 使用 | 理由 |
| --- | --- | --- |
| `GET /get_acc` | 使う | テレメトリの取得 |
| `GET /clear_buffer` | 起動時・再接続時のみ使う（パラメータで無効化可） | 起動前に溜まった古いサンプルを捨てる（モニター画面も読み込み時に同じ処理をしている） |
| `GET /command` | `enable_drive=true` のときだけ、§4.3 の ID に限って使う | 操縦 |
| `GET /` | **使わない** | アクセスすると MASTER 登録が起きる |

## 5. ROS インターフェース

### 5.1 メッセージ `robobike_msgs/msg/RobobikeTelemetry`（新規）

```text
# One control-loop sample (250 Hz) read from ROBOBIKE GET /get_acc.
std_msgs/Header header   # stamp: TIME_MS を ROS 時刻に換算した値（§6.3）、frame_id: パラメータ frame_id
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

### 5.2 Publish トピック

| トピック | 型 | QoS | 周期 | 内容 |
| --- | --- | --- | --- | --- |
| `/robobike/telemetry` | `robobike_msgs/msg/RobobikeTelemetry` | `qos_profile_sensor_data`（BEST_EFFORT、depth 5）。パラメータで RELIABLE / depth を変更可 | **1 サンプル 1 メッセージ（約 250 Hz）**。ポーリング（既定 20 Hz）ごとに、届いた全サンプルを時刻順にまとめて publish | テレメトリ本体 |
| `/robobike/drive_state` | `std_msgs/msg/String` | RELIABLE + TRANSIENT_LOCAL、depth 1 | 状態が変わったとき | 下りの状態（§6.6）: `DISABLED` / `STOPPED` / `STARTING` / `RUNNING` / `STOPPING` / `REVERSE` / `LOCKOUT` / `FAULT` |
| `/robobike/bridge/connected` | `std_msgs/msg/Bool` | RELIABLE + TRANSIENT_LOCAL、depth 1 | 状態が変わったとき | ROBOBIKE からテレメトリを取得できているか |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 既定 | 1 Hz | `robobike_bridge: telemetry`（受信レート、欠落数、HTTP エラー数、CSV 形式、最終受信からの経過時間）と `robobike_bridge: drive`（状態、最後に送ったコマンド、送信エラー数、捨てられたと推定されるコマンド数、`MOT_SPD`）のステータス |

### 5.3 Subscribe トピック

| トピック | 型 | QoS | 内容 |
| --- | --- | --- | --- |
| `/cmd_vel`（パラメータ `cmd_topic`） | `geometry_msgs/msg/Twist` | RELIABLE、depth 1 | `robobike_control` が調停した**唯一の走行指令**。`linear.x` [m/s] と `angular.z` [rad/s] だけを使う |

- 方策ノード（`robobike_policy`）の出力 `/policy/cmd_vel` は、ブリッジが直接は購読しない。`robobike_control` の AUTO モードを通した `/cmd_vel` を購読する。こうすることで、STANDBY 中のゼロ指令、指令のタイムアウト、人の操作による AUTO → TELEOP への切り替え、速度の上限を、既存の調停ノードに一元化できる。
- `enable_drive=false` のときは購読しない（`drive_state = DISABLED`）。

### 5.4 パラメータ

上り（テレメトリ）:

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

下り（操縦）:

| 名前 | 型 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `enable_drive` | bool | `false` | `true` のときだけ `/command` を送る。起動後の変更は受け付けない（再起動が必要） |
| `cmd_topic` | string | `/cmd_vel` | 走行指令のトピック |
| `cmd_timeout` | double | `0.5` | この秒数 `/cmd_vel` が届かなければ、`linear.x = 0` とみなす（＝停止） |
| `control_period` | double | `0.05` | 下りの状態機械を評価する周期 [s] |
| `start_linear` | double | `0.05` | `linear.x` がこの値 [m/s] を超えたら発進する |
| `stop_linear` | double | `0.02` | `linear.x` がこの値未満になったら停止する（`start_linear` より小さいこと） |
| `max_angular` | double | `1.0` | `angular.z` がこの値 [rad/s] のとき舵を最大（`value = ±100`）にする。`robobike_control` の `angular_limit` の既定値に合わせている |
| `steer_min_interval` | double | `0.1` | `bt_Str_S` を送る最小間隔 [s] |
| `steer_resend_period` | double | `0.5` | 値が変わらなくても、この周期で `bt_Str_S` を再送する（コマンドが捨てられた場合の回復用） |
| `allow_reverse` | bool | `false` | `true` のとき、停止中に `linear.x < -start_linear` なら `bt_BK` で後進する（後進は自立制御なし） |
| `speed_control` | bool | `false` | `true` のとき、走行中に `linear.x` に応じて `MOT_SPD` を `bt_A_Up_0` / `bt_A_Dn_0` で調整する（§6.7） |
| `max_linear` | double | `0.25` | `speed_control` で `MOT_SPD = 60` に対応させる速度 [m/s]（公式の走行速度の目安） |
| `min_drive_speed` | int | `30` | `speed_control` で使う `MOT_SPD` の下限（遅すぎると自立できないため）。0〜60 |
| `start_confirm_timeout` | double | `5.0` | `bt_F` を送ってから、テレメトリで走行（`SV_DRV > 0`）を確認するまでの待ち時間の上限 [s] |
| `stop_confirm_timeout` | double | `2.0` | `bt_S` を送ってから、テレメトリで停止（`SV_DRV == 0`）を確認するまでの待ち時間の上限 [s] |
| `stop_retries` | int | `3` | 停止を確認できないとき `bt_S` を再送する回数。使い切ったら `stp_all` を送る |
| `rearm_duration` | double | `1.0` | `LOCKOUT` を解除するのに必要な、`linear.x < stop_linear` が続く時間 [s] |

## 6. 内部動作

### 6.1 スレッドと HTTP 接続

- ROS のコールバック（タイマー、購読）はシングルスレッドの executor で処理し、コールバック内では HTTP 通信をしない。
- HTTP 通信は 2 本の専用ワーカースレッドで行う。
  - **テレメトリ用**: `/get_acc` のポーリングを担当する。
  - **コマンド用**: キューから `/command` を取り出して送る。
  - 2 本を分けるのは、片方の遅延やタイムアウトがもう片方を止めないようにするためである。スレッド間の受け渡しは `queue.Queue` とロックで行う。
- 各ワーカーは `http.client.HTTPConnection` を**使い回す（keep-alive）**。Pi から張るソケットは 2 本、スマホのブラウザが 1〜2 本使うため、ESP32 の上限 4 本に収まる。例外や応答異常が起きたら接続を閉じて作り直す。
- HTTP ライブラリは Python 標準ライブラリ（`http.client`）のみを使い、追加の依存を増やさない。

### 6.2 パース（上り）

- 応答本文を `\n` で分割し、空行は無視する。各行を `,` で分割する。
- 列数が 13 なら現行形式、8 なら旧形式として扱う。それ以外の列数や、`HEADER` が `a` でない行、数値に変換できない行は**その行だけ捨てて**、診断のカウンタ（`malformed`）を増やす。
- `nan`、`inf` もそのまま通す（ファームウェアの値を加工しない）。
- 形式が途中で変わった（OTA 更新など）場合は診断に記録し、新しい形式で処理を続ける。

### 6.3 タイムスタンプ（上り）

- `time_ms` には生値を必ず入れる。
- `header.stamp` は ROBOBIKE の時刻を ROS 時刻に換算した値とする。
  - 応答を受信した時刻 `t_recv`（ROS 時刻）と、その応答の最後のサンプルの `time_ms` から `offset = t_recv - time_ms` を求め、**直近 10 秒間の最小値**を時刻オフセットとして使う（ネットワーク遅延の影響を最も受けていない推定値）。
  - 各サンプルの `stamp = time_ms + offset`。こうすると、同じ応答に含まれるサンプル間の 4 ms 間隔が保たれる。
- `time_ms` が前回より小さくなった場合（ROBOBIKE の再起動やラップ）は、オフセットの推定をやり直し、診断に記録する。

### 6.4 欠落検出（上り）

- 連続するサンプルの `time_ms` の差が `1.5 × (1000 / 250) = 6 ms` を超えたら欠落とみなし、推定欠落数 `round(Δ / 4) - 1` を診断のカウンタに加算する。リングバッファの溢れや通信断が原因になる。
- ROBOBIKE の制御周期 250 Hz は定数としてコードに持つ（パラメータにはしない）。

### 6.5 エラー処理と再接続（上り）

| 事象 | 動作 |
| --- | --- |
| 接続失敗・タイムアウト・HTTP 200 以外 | WARN ログ（5 s で throttle）。接続を閉じ、0.5 s から最大 `reconnect_backoff_max` まで倍々に間隔を延ばして再試行する。**ノードは終了しない** |
| 応答は正常だが空（新しいサンプルがない） | 正常として扱う。`stale_timeout` を超えたら `connected=false` にする |
| 再接続に成功した | `clear_buffer_on_start` が true なら `/clear_buffer` を呼んでから再開する。`connected=true` にする |
| 不正な行 | §6.2 のとおり、その行だけ捨てる |
| パラメータ不正 | 起動時に `ValueError` を出して終了する（pi5_camera と同じ方針） |

### 6.6 下りの状態機械

ROBOBIKE の操縦は連続的な速度指令ではなく、「発進」「停止」「舵の目標値」という離散的なコマンドで行う。ブリッジは `/cmd_vel` を次の状態機械でコマンドに変換する。

**実際の走行状態はテレメトリの `SV_DRV` で判定する**（`> 0`: 前進中、`< 0`: 後進中、`== 0`: 停止中）。コマンドが捨てられた場合（§4.3）や、スマホから操作された場合にも、実際の状態に追従するためである。

`control_period` ごとに、`v = linear.x` を評価する（最後の受信から `cmd_timeout` を超えていれば `v = 0`）。

| 状態 | 条件 | 動作 → 次の状態 |
| --- | --- | --- |
| `STOPPED` | `v > start_linear` かつ `connected` | `bt_F` を 1 回だけ送る → `STARTING` |
| `STOPPED` | `v < -start_linear` かつ `allow_reverse` | `bt_BK` を送る → `REVERSE` |
| `STARTING` | `SV_DRV > 0` を確認した | → `RUNNING` |
| `STARTING` | `start_confirm_timeout` を超えた | WARN ログ → `LOCKOUT`（発進の自動再試行はしない） |
| `STARTING` | `v < stop_linear` | 発進シーケンスの完了を待ってから `bt_S` を送る → `STOPPING` |
| `RUNNING` | `v < stop_linear`、`cmd_timeout` 超過、`connected=false`、ノード終了のいずれか | `bt_S` を送る → `STOPPING` |
| `RUNNING` | 上記以外 | 舵（§6.7）、および `speed_control` なら速度（§6.7）を送る |
| `RUNNING` | ブリッジが停止を指令していないのに `SV_DRV == 0` になった（スマホの STOP や転倒判定） | 舵の目標値を 0 に戻す → `LOCKOUT` |
| `STOPPING` | `SV_DRV == 0` を確認した | → `STOPPED`（ただし `v >= stop_linear` のままなら `LOCKOUT`） |
| `STOPPING` | `stop_confirm_timeout` を超えた | `bt_S` を再送する。`stop_retries` 回で止まらなければ `stp_all` を送り、ERROR ログ → `FAULT` |
| `REVERSE` | `v > -stop_linear`、`cmd_timeout` 超過、`connected=false`、ノード終了のいずれか | `bt_BK` を送る（後進中の `bt_BK` は停止） → `STOPPING` |
| `LOCKOUT` | `v < stop_linear` が `rearm_duration` 続いた | → `STOPPED` |
| `FAULT` | — | コマンドを送らない。ノードを再起動するまで解除しない |

- `enable_drive=true` の間は **ROS が走行の主導権を持つ**。スマホで FORWARD を押しても、`v < stop_linear` であればブリッジがすぐに停止させる。スマホの STOP は常に有効で、押されると `LOCKOUT` になる。ROS 側で一度指令をゼロに戻すまで、ブリッジは再発進しない。
- `bt_F` は**停止中にしか送らない**。走行中の `bt_F` は「舵を 0 に戻す」という別の動作になるためである。
- 発進・停止シーケンスの実行中は、ファームウェアがほかのコマンドを捨てる。そのため、この間は舵や速度のコマンドを送らない。
- ノード終了時（SIGINT、`destroy_node`）は、`RUNNING` / `STARTING` / `REVERSE` であれば停止コマンドを送り、停止を確認するか `stop_confirm_timeout` が過ぎるまで待ってから終了する。

### 6.7 指令値の変換

**舵（`angular.z` → `bt_Str_S`）**

- `value = round(clamp(-angular.z / max_angular, -1, 1) × 100)`。ROS では `angular.z > 0` が左旋回、ROBOBIKE では `+` が右なので、符号を反転する。
- 送るのは `RUNNING` のときだけ。前回送った値から変わったとき（`steer_min_interval` 以上の間隔を空ける）と、`steer_resend_period` ごとに送る。
- 自転車なので、舵は「旋回の目標」であってヨーレートそのものではない。`angular.z` に対する実際のヨーレートは、速度と `STR_TURN` 次第で変わる。正確な対応付けは、テレメトリ（`GY_YAW`）を使って後で校正する（§10）。

**速度（`linear.x` → `MOT_SPD`、`speed_control=true` のときだけ）**

- 目標 `MOT_SPD = clamp(round(v / max_linear × 60), min_drive_speed, 60)`。
- `RUNNING` 中に、現在の `MOT_SPD`（`/command` の応答の 5 列目）と目標値の差を、`bt_A_Up_0` / `bt_A_Dn_0` で `control_period` ごとに 1 ずつ縮める。
- `bt_A_Up` / `bt_A_Dn`（ID 11 / 12）は**使わない**。これらは停止中でも駆動を開始してしまうためである。
- 副作用: `MOT_SPD` はファームウェアの保存値である。前進からの `bt_S` で NVS に保存されるため、変更した値はスマホでの操縦にも引き継がれる。ブリッジは起動時の `MOT_SPD` を覚えておき、停止を確認したあとに `bt_A_Up_0` / `bt_A_Dn_0`（停止中は値の変更だけで駆動しない）でその値に戻す。ただし、すでに NVS に保存された値は、次に保存されるまでそのまま残る。
- `speed_control=false`（既定）のときは、発進後の速度はスマホで設定した `MOT_SPD` のままになる（既定値 60 ＝ 約 0.25 m/s）。

### 6.8 安全要件のまとめ

1. 既定値は `enable_drive=false`（読み取り専用）。操縦は明示的に有効にしたときだけ行う。
2. `/cmd_vel` の途絶（`cmd_timeout`）、テレメトリの途絶（`connected=false`）、ノード終了のいずれでも停止コマンドを送る。停止はテレメトリで確認し、確認できなければ再送し、最後に `stp_all` を送る。
3. **残るリスク**: Pi の電源断、Pi の Wi-Fi 切断、コンテナの強制終了（SIGKILL）が起きると、ブリッジは停止コマンドを送れない。ファームウェアには通信途絶時のフェイルセーフが無いため、ROBOBIKE は走り続ける。**スマホの STOP ボタンを最終的な停止手段とし、走行中は人が必ず手元に置く**。根本的な対策はファームウェア側のウォッチドッグである（§10）。
4. 調整・設定系のコマンド（ID 13 以降）、`bt_A_Up` / `bt_A_Dn`、`bt_L` / `bt_R`、`GET /` は送らない。

## 7. ネットワーク設定（Raspberry Pi 5 側の必須事項）

ROBOBIKE の AP にはインターネット接続がない。さらに、ROBOBIKE はキャプティブポータル用の DNS サーバを持ち、**すべての名前解決に 192.168.4.1 を返す**。
wlan0 を既定の設定のまま接続すると、デフォルトルートや DNS が wlan0 側に取られ、apt や PC との通信が壊れるおそれがある。

- wlan0 の接続（NetworkManager の場合）には次の設定をする:
  `nmcli connection modify <ROBOBIKE-xxxx> ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled connection.autoconnect no`
  これで `192.168.4.0/24` だけが wlan0 経由になり、デフォルトルートと DNS は eth0 のまま保たれる。
- ROS 2 の DDS（Fast DDS）は既定で全インターフェースを使う。2 台しか接続できない ROBOBIKE の AP に DDS の multicast を流さないよう、Fast DDS のプロファイル XML（`interfaceWhiteList` に eth0 の IP を指定）で DDS を eth0 だけに限定することを推奨する。プロファイルは `docker/rpi5/fastdds_eth0_only.xml`（`ETH0_IPV4_ADDRESS` を eth0 の IP に書き換えて使う）で、Compose がコンテナの `/etc/robobike/fastdds_eth0_only.xml` にマウントするので、コンテナの各シェルで `export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/robobike/fastdds_eth0_only.xml` としてからノードを起動する（空の値を設定すると Fast DDS がエラーを出すため、Compose では既定値を設定していない）。
- コンテナは `network_mode: host` なので、Docker 側の追加設定は不要。
- 時刻: テレメトリの `header.stamp` は Pi の時計に換算した値で、[pi5_camera.md](pi5_camera.md) の画像と同じ時間軸になる。PC 側の時刻と比べる場合は、Pi と PC の時計を NTP（chrony など、eth0 経由）で同期する。

## 8. 起動例

```bash
# rpi5 コンテナ内
source /opt/ros/humble/setup.bash
colcon build --packages-select robobike_msgs pi5_camera robobike_bridge
source /ros2_ws/install/setup.bash
ros2 run robobike_bridge bridge_node                                # テレメトリのみ
ros2 run robobike_bridge bridge_node --ros-args -p enable_drive:=true   # 操縦も行う

# 確認（PC 側・Pi 側どちらでも）
ros2 topic hz /robobike/telemetry          # 約 250 Hz
ros2 topic echo /robobike/drive_state
ros2 topic echo /robobike/bridge/connected
```

`robobike_logger` の `record.launch.py` は、`/robobike/telemetry`、`/robobike/drive_state`、`/robobike/bridge/connected` も記録する。PC 側でも `robobike_msgs` をビルドしておくこと（記録にメッセージ型が必要）。

## 9. テスト方針（テスト先行）

`src/rpi5/robobike_bridge/test/` に pytest を作成し、失敗を確認してから実装する。rclpy / `robobike_msgs` が import できない環境では、[pi5_camera](pi5_camera.md) と同様に stub を `sys.modules` に入れて動かす。
状態機械と変換処理は ROS に依存しないモジュール（例: `robobike_bridge/drive.py`、`robobike_bridge/telemetry.py`）に分け、`robobike_control/mux.py` と同じように単体でテストできるようにする。

| テスト | 内容 |
| --- | --- |
| パーサ | 13 列・8 列の正常行、空行、列数不正、`HEADER` 不正、数値不正、CRLF。8 列では拡張フィールドが NaN になること |
| パラメータ検証 | `poll_period` / `http_timeout` / `cmd_timeout` などが 0・負・NaN・inf のとき、`poll_period >= 5.0` のとき、`stop_linear >= start_linear` のとき、`min_drive_speed` が範囲外のときに `ValueError` |
| タイムスタンプ・欠落 | 応答内のサンプル間隔が 4 ms で保たれること。最小オフセットが使われること。`time_ms` が巻き戻ったら推定がやり直されること。欠落数が計算されること |
| 状態機械 | §6.6 の全ての遷移。とくに: `bt_F` を停止中に 1 回だけ送ること、`cmd_timeout` と `connected=false` で停止すること、停止の再送と `stp_all`、外部からの停止で `LOCKOUT` になること、`rearm_duration` で解除されること、`allow_reverse=false` では後進しないこと |
| 指令値の変換 | `angular.z` の符号反転と飽和、送信間隔と再送、`speed_control` で `bt_A_Up_0` / `bt_A_Dn_0` だけを使うこと、停止後に `MOT_SPD` を元に戻すこと |
| HTTP 結合 | `http.server` によるフェイク ROBOBIKE（`/get_acc`、`/clear_buffer`、`/command`。`SV_DRV` の変化を模擬する）に対して: publish 件数・順序、`enable_drive=false` では `/command` を送らないこと、**`/` と禁止 ID を送らないこと**、タイムアウト・500 応答・切断からの再接続、ノード終了時の停止 |
| 実データ | 上流リポジトリ `doc/*.csv` の実データ（8 列形式・13 列形式）を再生して、全行がパースされること |
| 実機（手動） | Pi 5 + ROBOBIKE 実機で、(1) 2 台目の Pi からの `/command`（まず `only_data`、次に車体を持ち上げた状態で `bt_F` / `bt_S`）が効くこと、(2) MASTER がスマホのままであること、(3) `ros2 topic hz` で約 250 Hz、(4) `/cmd_vel` の途絶・ブリッジの終了で停止すること、(5) スマホの STOP で `LOCKOUT` になること |

## 10. 範囲外・今後の検討事項

- **ファームウェア側のウォッチドッグ**（一定時間 `/command` を受けなければ `bt_S` 相当の処理を行う）。Pi の電源断や Wi-Fi 切断にも対応できる唯一の根本対策なので、上流への提案を検討する。
- `angular.z` → `bt_Str_S`、`linear.x` → `MOT_SPD` の対応関係を、テレメトリ（`GY_YAW`）と実測速度で校正すること。
- 方策の学習で使う行動表現を、`Twist` ではなく ROBOBIKE のコマンド（発進／停止、舵 -100〜100、`MOT_SPD`）そのものにするかどうか。
- 設定値レコード（`b` 行）の ROS への publish。
- `sensor_msgs/Imu` 形式での再 publish（軸の向き・符号の定義と rad/s への換算が必要）。
- 複数台の ROBOBIKE（AP が別々）への同時接続。
