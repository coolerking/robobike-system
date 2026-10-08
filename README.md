# robobike-system

Raspberry Pi 5と外部PCをROS 2 Humbleで接続し、Robobikeのテレオペ、
走行データ収集、SmolVLAによる模倣学習・自律制御を開発するモノレポです。
固定カメラ等への拡張も想定しています。ROSノードはPython / `rclpy`、
ML環境はROS非依存のPyTorchコンテナに分離します。

> **開発途中です。**
> USBカメラ、Joy変換、制御Mux、rosbag記録、ROBOBIKE実機とのHTTPブリッジ
> （テレメトリ取得と`/cmd_vel`による操縦）は実装済みですが、SmolVLA推論サービス、
> データセット変換、ファインチューニングは未実装です。policyノードは走行指令を生成しません。
> ブリッジは既定で読み取り専用で、`enable_drive:=true`を指定したときだけ車体を動かします。

## 構成

```text
robobike-system/
├── .gitignore
├── README.md
├── doc/
│   └── spec/                  # パッケージ仕様書（pi5_camera.md、robobike-bridge.md）
├── docker/
│   ├── rpi5/                  # ARM64 ROS環境（fastdds_eth0_only.xml: DDSをeth0に限定）
│   ├── pc_ros/                # x86_64 ROS + NVIDIA環境
│   └── pc_ml/                 # ROS非依存のPyTorch + NVIDIA環境
├── src/
│   ├── common/
│   │   └── robobike_msgs/      # ament_cmake、カスタムmsg
│   │       └── msg/RobobikeTelemetry.msg
│   ├── rpi5/
│   │   ├── pi5_camera/         # USBカメラ → sensor_msgs/Image
│   │   └── robobike_bridge/    # ROBOBIKE HTTP ⇄ ROS（テレメトリ、/cmd_vel → 操縦コマンド）
│   └── pc/
│       ├── robobike_teleop/    # Joy → Twist（デッドマン付き）
│       ├── robobike_policy/    # 外部SmolVLA推論へのROSアダプタ
│       ├── robobike_control/   # STANDBY / TELEOP / AUTO調停
│       └── robobike_logger/    # launch/record.launch.py
└── ml/
    ├── visualization/         # app.py、Streamlit + rerun-sdk
    ├── data_processing/       # parse_bag.py、rosbags
    └── training/              # finetune.py、LeRobot SmolVLA
```

各Dockerディレクトリに`Dockerfile`と`docker-compose.yml`、各MLディレクトリに
`requirements.txt`を配置しています。Python ROSパッケージは公式の
`ros2 pkg create --build-type ament_python`構成に基づき、`package.xml`、
`setup.py`、`setup.cfg`、ament resource marker、Pythonモジュールを持ちます。
`robobike_msgs`は`CMakeLists.txt`を持つインターフェースパッケージで、
ROBOBIKEのテレメトリ`RobobikeTelemetry.msg`を定義しています。msgを追加するときは
`rosidl_generate_interfaces`に追記してください。`robobike_bridge`と`robobike_logger`が
依存するため、PiとPCの両方で`robobike_msgs`をビルドします。

## ノードとトピック

```text
USB camera ── pi5_camera ── /camera/image_raw ── robobike_policy
                                                    │
Joy ── robobike_teleop ── /teleop/cmd_vel             /policy/cmd_vel
                              │                     │
                              └── robobike_control ─┘
                                        │
                           /cmd_vel ── robobike_bridge ──(HTTP, wlan0)── ROBOBIKE
                                        │
              /robobike/telemetry, /robobike/drive_state, /robobike/bridge/connected
                                        │
                                   rosbag record
```

| コンポーネント | 入力 | 出力・役割 |
| --- | --- | --- |
| `pi5_camera` | USBカメラ（既定`/dev/v4l/by-id`を走査、無ければ`/dev/video0`） | `/camera/image_raw` (`sensor_msgs/Image`, `bgr8`, sensor-data QoS)、ROS時刻とframe ID付き |
| `robobike_bridge` | ROBOBIKEの`GET /get_acc`、`/cmd_vel` (`geometry_msgs/Twist`、`enable_drive:=true`のときだけ購読) | `/robobike/telemetry` (`robobike_msgs/RobobikeTelemetry`、約250 Hz)、`/robobike/drive_state` (`String`)、`/robobike/bridge/connected` (`Bool`)、`/diagnostics`。`/cmd_vel`を発進・停止・舵の`GET /command`に変換する。仕様は[doc/spec/robobike-bridge.md](doc/spec/robobike-bridge.md) |
| `robobike_teleop` | `/joy` (`sensor_msgs/Joy`) | `/teleop/cmd_vel` (`Twist`) |
| `robobike_policy` | `/policy/enable` (`std_msgs/Bool`)、`/camera/image_raw` | enableがTrueかつ画像が新しい場合だけ`infer()`を呼び、結果があれば`/policy/cmd_vel` (`Twist`)へ出力 |
| `robobike_control` | `/control/mode` (`std_msgs/String`)、2系統のTwist | `/cmd_vel`、`/policy/enable`、`/control/state` (`String`) |
| `robobike_logger` | 上記トピック | SQLite3 rosbag（`metadata.yaml`、`.db3`） |

`/policy/enable`と`/control/state`はreliable / transient-local / depth 1で配信します。
policyノードは後から起動しても現在のenableを取得できます。
`/policy/enable`の唯一の配信者は制御Muxとしてください。

### 調停と安全性

- 起動状態は**STANDBY**。ゼロTwistを20 Hzで送信します。
- **TELEOP**では手動指令のみ、**AUTO**ではpolicy指令のみを採用します。
- AUTO中、手動指令の`linear.x`または`angular.z`の絶対値が
  `human_deadband`を超えると直ちにTELEOPへ切り替え、推論を無効化します。
  AUTOへの自動復帰はありません。ゼロの手動指令はAUTOを解除しません。
- 状態変更時には旧指令を破棄します。対象入力が0.5秒以上途絶した場合は停止指令を出し、
  新しい入力で再開します（状態は維持）。非有限値は拒否し、その入力の旧指令も破棄します。
- 出力は平面走行用の`linear.x` / `angular.z`のみです。既定上限は0.5 m/s、
  1.0 rad/sです。実機に適した値へ必ず変更してください。
- Joyはボタン0を押している間だけ有効。軸1が前後、軸0が旋回です。
  ボタンを離す、不正なJoyを受け取る、Joyが途絶する場合はゼロ指令を送ります。
  AUTO中にデッドマンを離すだけではAUTOは停止しません。停止はSTANDBY指示で行います。

このMuxは機能安全を保証しません。PC停止、DDS切断、遅延・滞留メッセージを
PC側の受信時刻だけで完全には検出できません。実機通信を実装する際は
**Pi/モーター側の独立したウォッチドッグ、速度制限、物理非常停止**を必須としてください。
テストは車輪を浮かせる等、走行できない状態から始めてください。

主なROSパラメータ（`--ros-args -p name:=value`）:

| ノード | パラメータ（既定値） |
| --- | --- |
| camera | `device=/dev/v4l/by-id`, `camera_id=""`, `fps=15.0`（Hz）, `frame_id=camera`, `topic=/camera/image_raw`, `reopen_after_failures=30` |
| bridge | `base_url=http://192.168.4.1`, `poll_period=0.05`, `enable_drive=false`, `cmd_timeout=0.5`, `start_linear=0.05`, `stop_linear=0.02`, `max_angular=1.0`, `allow_reverse=false`, `speed_control=false`（全パラメータは仕様書§5.4） |
| teleop | `linear_axis=1`, `angular_axis=0`, `deadman_button=0`, `linear_scale=0.5`, `angular_scale=1.0`, `joy_timeout=0.5` |
| policy | `image_timeout=0.5` |
| control | `command_timeout=0.5`, `linear_limit=0.5`, `angular_limit=1.0`, `human_deadband=0.05` |

## 必要環境

- Pi: Raspberry Pi 5、64-bit Linux、USBカメラ。
- PC: Linux x86_64、NVIDIA GPU/ドライバ、NVIDIA Container Toolkit、ジョイスティック。
- 両方: Docker EngineとDocker Compose v2、同一LAN。ROS用コンテナは
  `network_mode: "host"`を使用し、DDSのUDP/multicastが通る必要があります。
- `ROS_DOMAIN_ID`は両デバイスで一致させます（既定42）。
  `ROS_LOCALHOST_ONLY=0`と`rmw_fastrtps_cpp`をComposeで設定しています。
  信頼できる隔離LANで利用してください。host networkingは認証や暗号化を提供しません。

**ARM64イメージについて:** 指定の`osrf/ros:humble-desktop-full`はamd64専用です。
Pi用Dockerfileはこのベースを既定のbuild argとして保持しますが、
Pi用Composeは公式のARM64対応`ros:humble-ros-base`へ切り替え、
`ros-humble-desktop-full`を追加インストールします。Piでは必ずCompose経由で
ビルドするか`--build-arg ROS_IMAGE=ros:humble-ros-base`を指定してください。
desktop-fullは大きいため、Pi側の空き容量に注意してください。
PC ROSは指定イメージ、PC MLは`pytorch/pytorch:latest`を使用します。
`latest`は可変なので、本番運用時は動作確認済みタグ/digestへ固定してください。
MLの依存解決によりベースのPyTorchが変更される可能性があるためCUDA互換性も確認します。

## 起動とビルド

以下では各ホスト上のリポジトリの**絶対パス**を設定します。

```bash
export ROBOBIKE_ROOT=/absolute/path/to/robobike-system
export ROS_DOMAIN_ID=42
```

Piで:

```bash
export CAMERA_DEVICE=/dev/video0
docker compose -f "$ROBOBIKE_ROOT/docker/rpi5/docker-compose.yml" up -d --build
docker compose -f "$ROBOBIKE_ROOT/docker/rpi5/docker-compose.yml" exec rpi5 bash
```

コンテナ内で:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robobike_msgs pi5_camera robobike_bridge
source /ros2_ws/install/setup.bash
ros2 run pi5_camera camera_node
# 別のコンテナシェルでも上記2つのsetup.bashをsourceしてから:
ros2 run robobike_bridge bridge_node                                  # テレメトリのみ
ros2 run robobike_bridge bridge_node --ros-args -p enable_drive:=true # 操縦も行う
```

ブリッジを使う前に、スマホを1台目としてROBOBIKEのAPに接続してニュートラル調整を済ませ、
Piのwlan0を2台目として接続します（wlan0の設定は仕様書§3・§7）。
`enable_drive:=true`で走行させる間は、スマホのSTOPボタンを最終的な停止手段として必ず手元に置きます。

DDSをeth0に限定する場合（推奨）は、`docker/rpi5/fastdds_eth0_only.xml`の
`ETH0_IPV4_ADDRESS`をPiのeth0のIPアドレスに書き換え、コンテナの各シェルで
`export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/robobike/fastdds_eth0_only.xml`としてからノードを起動します
（Composeがこのファイルを`/etc/robobike/`にマウントします）。

PCで:

```bash
export JOY_DEVICE=/dev/input/js0
docker compose -f "$ROBOBIKE_ROOT/docker/pc_ros/docker-compose.yml" up -d --build
docker compose -f "$ROBOBIKE_ROOT/docker/pc_ros/docker-compose.yml" exec pc_ros bash
```

コンテナ内で:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robobike_msgs robobike_teleop robobike_policy robobike_control robobike_logger
source /ros2_ws/install/setup.bash
ros2 run robobike_control control_node
# 以降は別々のコンテナシェルでsetup.bashをsourceして起動:
ros2 run joy joy_node
ros2 run robobike_teleop teleop_node
ros2 run robobike_policy policy_node
```

ホストの`src/`はコンテナの`/ros2_ws/src`にread-onlyでマウントしています。
ソース編集はホストで行い、`colcon build`はコンテナ内で行います。
通常ビルドを使い、read-onlyソースを変更する可能性がある`--symlink-install`は使いません。
`build/`, `install/`, `log/`はデバイス別Composeの名前付きボリュームに保存されます。
`down`では残り、`down -v`で削除されます。両ホストで独立してビルドしてください。
依存追加時は`package.xml`とDockerfileを更新するか、
`rosdep update && rosdep install --from-paths /ros2_ws/src --ignore-src -r -y`を利用します。
maintainerの`example.com`アドレスは公開用のダミーで、プロジェクト管理者に置換してください。

GPUや入力デバイスがない開発用PCでは、ComposeのGPU予約と対応する`devices`を
外してから起動します。Piの`CAMERA_DEVICE`、PCの`JOY_DEVICE`は存在するデバイスを
指定してください。GUI表示設定（DISPLAY、Xauthority等）は必要に応じて別途設定します。

### モード操作・確認

PC ROSコンテナ内で:

```bash
ros2 topic pub --once /control/mode std_msgs/msg/String "{data: TELEOP}"
ros2 topic echo /cmd_vel
ros2 topic echo /control/state --qos-durability transient_local
ros2 topic pub --once /control/mode std_msgs/msg/String "{data: AUTO}"
# 停止:
ros2 topic pub --once /control/mode std_msgs/msg/String "{data: STANDBY}"
ros2 topic list
ros2 topic hz /camera/image_raw
```

現状のAUTOでは`infer()`が`None`を返すため走行しません。
外部PC MLとのRPC/IPCプロトコルは未定義です。実装時はROS executorをブロックしない
非同期通信、タイムアウト、推論結果の鮮度、画像・ロボット状態からモデル入力への変換、
モデルactionからTwistへの変換を定義してください。ROS環境へPyTorchを直接混在させない
構成を想定しています。

## rosbag記録とML

PC ROSコンテナで記録（Ctrl-Cで正常終了）:

```bash
ros2 launch robobike_logger record.launch.py output:=/data/bags/session_001
```

`output`は毎回未使用のディレクトリを指定してください。ホストの`bags/`へ保存されます。
画像、Joy、両入力指令、実際の`/cmd_vel`、モード、policy enable、ROBOBIKEのテレメトリ・
ブリッジの状態を記録します。
画像の容量増大に注意し、再生時は実機bridgeを接続しないでください。

PCでMLコンテナを起動:

```bash
docker compose -f "$ROBOBIKE_ROOT/docker/pc_ml/docker-compose.yml" up -d --build
docker compose -f "$ROBOBIKE_ROOT/docker/pc_ml/docker-compose.yml" exec pc_ml bash
```

MLコンテナ内で:

```bash
python /workspace/ml/data_processing/parse_bag.py /data/bags/session_001
streamlit run /workspace/ml/visualization/app.py --server.address 0.0.0.0
# 別シェルでGPU確認:
python -c "import torch; print(torch.cuda.is_available())"
python -m pip check
python /workspace/ml/training/finetune.py --help
```

StreamlitはPCの`http://127.0.0.1:8501`で開きます（公開ポートはループバック限定）。
MLコンテナにはROSもhost networkingも必要ありません。rosbagsがbagディレクトリ内の
`.db3`を読み、現在のスクリプトはトピック名・型・メッセージ数のみ表示します。
UIは案内表示とrerun初期化のみ、学習スクリプトはCLIの空枠で、実行すると明示的に
`NotImplementedError`になります。

今後の実装順序:

1. 画像と指令の時刻同期、欠損除去、TELEOP区間抽出、学習/検証分割。
2. LeRobotデータセットのobservation/action schema、単位・正規化・制御周期の定義。
3. キュレーションUIとrerun時系列可視化の接続。
4. LeRobotのSmolVLA設定でファインチューニング、評価、チェックポイント保存。
5. 外部推論サービスとpolicyアダプタの接続、実機ウォッチドッグ、安全試験。

## 開発時の検証

Muxの回帰テストはPython標準の`unittest`のみで実行でき、ROSは不要です。
ホスト上で:

```bash
PYTHONPATH="$ROBOBIKE_ROOT/src/pc/robobike_control" \
  python3 -m unittest discover -s "$ROBOBIKE_ROOT/src/pc/robobike_control/test" -v
python3 -m pytest "$ROBOBIKE_ROOT/src/rpi5/pi5_camera/test" -v
python3 -m pytest "$ROBOBIKE_ROOT/src/rpi5/robobike_bridge/test" -v
docker compose -f "$ROBOBIKE_ROOT/docker/rpi5/docker-compose.yml" config --quiet
docker compose -f "$ROBOBIKE_ROOT/docker/pc_ros/docker-compose.yml" config --quiet
docker compose -f "$ROBOBIKE_ROOT/docker/pc_ml/docker-compose.yml" config --quiet
```

カメラノードとブリッジのテストは、rclpy・cv2・cv_bridge・メッセージパッケージが無い環境ではstubを注入して実行されます。
ブリッジのテストはフェイクのROBOBIKE（`http.server`）に対して動きます。
Piコンテナ内では`colcon test --packages-select pi5_camera robobike_bridge && colcon test-result --verbose`で実ROSを使って実行できます。

ROSコンテナ内では上記ビルド後に`ros2 pkg executables`、
`ros2 launch robobike_logger record.launch.py --show-args`でインストールを確認できます。
実機接続前にJoy・policy入力を模擬し、STANDBY停止、手動割り込み、
入力途絶時の停止を確認してください。

`ml/data/`、`bags/`、`.db3`、`.mcap`、`.pt`、`.pth`、`.safetensors`および
ROS/Pythonビルド成果物はGit管理から除外します。データや重みは別ストレージで
バージョン管理してください。ライセンスは[MIT](LICENSE)です。
