# 車輪オドメトリの融合

`wheel.enabled: true` にすると、車輪速度をLiDARの点対平面残差と同じ
反復誤差状態EKFへ入力する。出力TFだけを補正する方式ではなく、融合後の姿勢で
スキャンを地図へ登録する。直線壁に沿う方向など、LiDARで拘束できない移動を補う。

Siriusのシミュ設定 `params/fast_lio_sirius_sim.yaml` では有効。
その他の設定では既定で無効。

- 入力は `nav_msgs/Odometry` の `twist`。絶対姿勢・位置とodomの原点は使わない。
- `twist.linear` は `child_frame_id` の座標系、単位m/s。
  `twist.angular.z` は同じ座標系での旋回速度、単位rad/s。
- 差動二輪の横・上下速度ゼロも車体座標系での拘束として使用する。
- `wheel.child_frame` と入力の `child_frame_id` の一致が必要。
- IMUから車体原点への位置・回転を `imu_to_base_translation` と
  `imu_from_base_rotation` に指定する。旋回時のレバーアーム速度も補正する。
- スキャン終了時刻に車輪速度を補間する。前後のデータが
  `wheel.max_age` 秒以内にない場合は車輪観測を入れず、従来のLIOで継続する。
- 共分散はtwistの対角成分を使用。`velocity_std` と `yaw_rate_std` は標準偏差の下限。
  スリップや段差の多い環境では車輪の共分散・下限値を大きくする。
- FAST-LIO自身の `/Odometry` を入力してはならない。

ログの `Wheel fusion active` は実際のスキャン時刻に対応する車輪データが使用できた印。
`used=N/M scans` は地図更新時の車輪融合の利用回数と処理スキャン数。
`Wheel fusion inactive` やchild frame警告が続く場合はトピック・時刻・フレームを確認する。
車輪も滑る環境や、既に壊れた地図を修復する機能ではない。

シミュのIMUと瞬間点群は同時刻に配信されるため、スキャン終端と同時刻のIMUも
同期区間に含める。IMUが得られない区間は地図へ登録せず、IMU処理の未完了時にも
前回の処理済み点群を使い回さない。

検証: `ctest --test-dir build/fast_lio --output-on-failure -R 'wheel_odometry|imu_processing'`
（ワークスペースのルートから実行）。時刻補間、古い入力・NaNの拒否、レバーアームと
回転のヤコビアン、EKFの両計算経路、LiDAR拘束ゼロ時の更新、
直線壁＋床のみの10m走行でのIMUバイアスによる縮退を検証する。
加えて同時刻IMU＋点群の連続100スキャンの同期、未来のIMUの保持、
空のIMU区間と初期化中の処理で前回点群が残らないことを検証する。
