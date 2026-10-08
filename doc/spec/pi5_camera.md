# pi5_camera 仕様書

| 項目 | 内容 |
| --- | --- |
| パッケージ | `src/rpi5/pi5_camera`（ament_python）、実行ファイル `camera_node`（`pi5_camera.camera_node:main`） |
| 実行環境 | Raspberry Pi 5 / `docker/rpi5` コンテナ（ROS 2 Humble、`network_mode: host`） |
| 機能 | Pi に USB 接続したカメラから OpenCV（`cv2.VideoCapture`）でフレームを取得し、`sensor_msgs/msg/Image` として一定周期で publish する |
| 状態 | **実装済み**（PR #2）。本書は現行実装を仕様として記述したもの |

## 1. 全体構成での位置付け

```text
[USB camera] ── /dev/videoN ── docker rpi5 ── pi5_camera ── /camera/image_raw ──(eth0, DDS)──▶ [PC] robobike_policy
                                                                                                    robobike_logger（rosbag）
```

- `robobike_policy` は `/camera/image_raw` を購読し、最後に画像を受信してから `image_timeout`（既定 0.5 s、PC 側の受信時刻で判定）以内の場合だけ推論する。
  - カメラが止まると方策の出力が止まり、`robobike_control` の `command_timeout` によって `/cmd_vel` がゼロになる。その結果、`robobike_bridge` が停止コマンドを送る（[robobike-bridge.md](robobike-bridge.md) §6.6）。
- 運用手順は [robobike-bridge.md](robobike-bridge.md) §3 に従う。カメラノードは手順 5 で、ブリッジより先に起動する。

## 2. 起動

```bash
# Pi ホスト
export CAMERA_DEVICE=/dev/v4l/by-id/usb-<vendor>_<model>-video-index0   # 推奨（未指定なら /dev/video0）
docker compose -f "$ROBOBIKE_ROOT/docker/rpi5/docker-compose.yml" up -d --build
docker compose -f "$ROBOBIKE_ROOT/docker/rpi5/docker-compose.yml" exec rpi5 bash

# コンテナ内
source /opt/ros/humble/setup.bash
colcon build --packages-select robobike_msgs pi5_camera robobike_bridge
source /ros2_ws/install/setup.bash
ros2 run pi5_camera camera_node
ros2 run pi5_camera camera_node --ros-args -p fps:=10.0     # パラメータ指定の例
```

## 3. ROS インターフェース

### 3.1 Publish トピック

| トピック | 型 | QoS | 周期 | 内容 |
| --- | --- | --- | --- | --- |
| `/camera/image_raw`（パラメータ `topic`） | `sensor_msgs/msg/Image` | `qos_profile_sensor_data`（BEST_EFFORT、depth 5） | `fps` [Hz]（既定 15） | `encoding = bgr8`。解像度はカメラ／OpenCV の既定値のまま。`header.stamp` はフレームを読み出した時点の ROS 時刻（Pi の時計）、`header.frame_id` はパラメータ `frame_id` |

### 3.2 Subscribe トピック

なし。

### 3.3 パラメータ

| 名前 | 型 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `device` | string | `/dev/v4l/by-id` | 開くデバイス。by-id のシンボリックリンク、`/dev/videoN`、またはディレクトリ（中の候補を走査する）を指定する。解決方法は §4.1 |
| `camera_id` | string | `""` | by-id のエントリ名に対する部分一致の文字列。複数のカメラから 1 台を選ぶのに使う |
| `fps` | double | `15.0` | publish する周期 [Hz]。タイマーの周期は `1 / fps`。有限かつ正でなければ `ValueError` |
| `frame_id` | string | `camera` | `header.frame_id` |
| `topic` | string | `/camera/image_raw` | publish するトピック名 |
| `reopen_after_failures` | int | `30` | フレームの読み出しがこの回数連続で失敗したら、デバイスを開き直す。1 未満なら `ValueError` |

- `fps` はタイマーの周期であって、カメラ側のフレームレートは設定しない（`CAP_PROP_FPS` などは変更しない）。カメラの実際のフレームレートが `fps` より低いと、`read()` で待たされるぶん publish の周期が延びる。

## 4. 内部動作

### 4.1 デバイスの解決 `resolve_device(device_param, camera_id="")`

`/dev/videoN` の番号は、USB の抜き差しや起動順で変わることがある。そのため、変わらない名前の `/dev/v4l/by-id/*` を使って実デバイスを特定する。

1. `device_param` が存在する**ファイル**（シンボリックリンクを含む）なら、`os.path.realpath()` で実デバイスに解決したパスを返す。
2. そうでなければ、by-id のディレクトリを走査する。走査するのは、`device_param` がディレクトリならそのディレクトリ、それ以外なら `/dev/v4l/by-id`。リンク切れのエントリは除外する。
   - `camera_id` が空なら、名前が `video-index0` で終わる最初のエントリ（名前順）を選ぶ。
   - `camera_id` があれば、`video-index0` のエントリのうち名前に `camera_id` を含むものを選ぶ。無ければ全エントリから同じように選ぶ。
   - 見つかったら、`realpath()` で解決したパスを返す。
3. 見つからなかった場合:
   - `device_param` がディレクトリ、または `/dev/v4l/by-id` そのものなら、`/dev/video0` を返す。
   - それ以外なら、`device_param` をそのまま返す。

**コンテナでの注意**: 現行の `docker/rpi5/docker-compose.yml` は、ホストの `${CAMERA_DEVICE:-/dev/video0}` をコンテナの `/dev/video0` としてだけ渡す。そのため**コンテナ内には `/dev/v4l/by-id` が無く**、既定の設定では手順 3 により `/dev/video0` が開かれる。
番号の変動に強くするには、ホスト側で `CAMERA_DEVICE` に by-id のパスを指定する。Docker は起動時にシンボリックリンクを実デバイスに解決して `/dev/video0` に割り当てる。
コンテナ内で by-id による選択（`camera_id`）を使う場合は、Compose に `/dev/v4l` のマウントと、該当する `/dev/videoN` を追加する必要がある。現状の Compose は追加していない。

### 4.2 オープンと再オープン

- 起動時に `open_capture()` を 1 回呼ぶ。開けなかった場合は WARN ログ（`Cannot open camera: <device> (will retry)`）を出し、**ノードは終了しない**。
- `open_capture()` は、そのたびに `resolve_device()` をやり直す。そのため、再接続で `/dev/videoN` の番号が変わっていても追従する（by-id を参照できる場合）。
- `destroy_node()` で、開いているデバイスを解放する。

### 4.3 フレームの publish（タイマーのコールバック）

| 事象 | 動作 |
| --- | --- |
| `read()` に成功した | `CvBridge.cv2_to_imgmsg(frame, "bgr8")` で変換し、`header` を付けて publish する。連続失敗回数を 0 に戻す |
| `read()` に失敗した、またはデバイスが開いていない | publish しない。WARN ログ（`Camera frame unavailable`、5 s で throttle）を出し、連続失敗回数を増やす。回数が `reopen_after_failures` に達したら 0 に戻して `open_capture()` をやり直す（既定値では 15 Hz × 30 回 ≒ 2 s ごと） |

- `read()` はタイマーのコールバック内で同期的に呼ぶ（シングルスレッドの executor）。

## 5. ネットワークと時刻

- 画像は無圧縮の `bgr8` なので、DDS の帯域を大きく使う（例: 640×480 × 3 B × 15 Hz ≒ 13.8 MB/s）。PC へは有線 LAN（eth0）で送る前提である。
- [robobike-bridge.md](robobike-bridge.md) §7 のとおり、Pi の wlan0 は ROBOBIKE の AP（2 台までしか接続できず、帯域も小さい）につながる。画像の DDS トラフィックが wlan0 に流れないよう、Fast DDS のプロファイル（`docker/rpi5/fastdds_eth0_only.xml`）で DDS を eth0 に限定することを推奨する。プロファイルは `docker/rpi5` のコンテナ全体に効くので、`robobike_bridge` と共通の設定になる。
- `header.stamp` は Pi の時計による。`robobike_bridge` のテレメトリも Pi の時計に換算した値なので、Pi 上の 2 つのストリームは同じ時間軸で突き合わせられる。
  - PC 側で受信時刻（rosbag の記録時刻など）と比べる場合は、Pi と PC の時計を NTP（chrony など、eth0 経由）で同期しておく。
  - `robobike_policy` の鮮度判定は PC の単調時計による受信時刻なので、時計の同期には依存しない。

## 6. テスト

`src/rpi5/pi5_camera/test/test_camera_node.py`（pytest / unittest）。

- rclpy・cv2・cv_bridge・sensor_msgs が import できない環境では、stub を `sys.modules` に入れて動く。
- `cv2.VideoCapture` は `unittest.mock` でモックし、by-id は一時ディレクトリ上のシンボリックリンクで模擬する。

| 対象 | 主なテスト |
| --- | --- |
| `resolve_device` | by-id のリンクの解決、`/dev/videoN` 直接指定、存在しないパスからの走査、ディレクトリ指定、`camera_id` による部分一致、リンク切れの除外、候補が無いときの戻り値（`/dev/video0` または param） |
| パラメータ | `fps` が 0・負・NaN のとき `ValueError`、`reopen_after_failures` が 1 未満のとき `ValueError`、タイマー周期 = `1 / fps`、既定値、`topic` |
| publish | `read()` 失敗時は publish しないこと、成功時は `frame_id` と `stamp` を付けて publish すること |
| 失敗からの回復 | オープン失敗で終了しないこと、失敗が続いたら開き直すこと、成功で失敗回数が戻ること、開き直すときにデバイスを解決し直すこと |

実行方法:

```bash
python3 -m pytest src/rpi5/pi5_camera/test -v                                 # ROS なしの開発機
colcon test --packages-select pi5_camera && colcon test-result --verbose       # rpi5 コンテナ内
```

`setup.py` の `tests_require=["pytest"]` が無いと、`colcon test` は 0 件のテストしか実行しない。

## 7. 範囲外・今後の検討事項

- 解像度・カメラ側のフレームレート・露出の設定（`width` / `height` などのパラメータ）。
- 圧縮画像（`sensor_msgs/msg/CompressedImage`、`image_transport`）での publish。帯域と rosbag の容量を減らすため。
- `camera_info`（内部パラメータ）の publish と、TF のフレーム定義（`camera` と ROBOBIKE の `robobike` フレームの関係）。
- コンテナ内で by-id を使うための Compose の変更（§4.1）。
