# tagslam

AprilTag 観測とボディオドメトリを統合し、地図・軌跡を生成する ROS2 ノード群です。

## Active API

### タグ検出結果
- Topic：`cameras.<cam>.tag_topic`（例：/tagslam/tag/cam0）
- Node：(prefix)_sync_and_detect
- Type： apriltag_msgs/msg/ApriltagArray
- Note：画像パラメータで指定した tag_topic に detector 出力を publish
- Usage：
```
ros2 topic echo /tagslam/tag/cam0
```

### 同期済みオドメトリ
- Topic：`<body>.odom_topic` + `_synced`
- Node：(prefix)_sync_and_detect
- Type： nav_msgs/msg/Odometry
- Note：入力オドメトリを画像タイムスタンプに合わせて再 publish
- Usage：
```
ros2 topic echo /odom/base_synced
```

### 最適化後ボディオドメトリ
- Topic：/tagslam/odom/body_<body>
- Node：(prefix)_tagslam
- Type： nav_msgs/msg/Odometry
- Note：グラフ最適化後の Pose。`publishAck` 有効時はフレーム毎に更新
- Usage：
```
ros2 topic echo /tagslam/odom/body_robot
```

### 最適化後経路
- Topic：/tagslam/path/body_<body>
- Node：(prefix)_tagslam
- Type： nav_msgs/msg/Path
- Note：`/tagslam/odom/body_*` の履歴を Path で公開
- Usage：
```
ros2 topic echo /tagslam/path/body_robot
```

### オペレーションコマンド
- Topic：/replay, /dump, /plot（std_srvs/srv/Trigger）
- Node：(prefix)_tagslam
- Type： std_srvs/srv/Trigger
- Note：`ros2 service call /replay ...` 等で再生・ダンプ・グラフ出力をトリガ
- Usage：
```
ros2 service call /dump std_srvs/srv/Trigger "{}"
```
