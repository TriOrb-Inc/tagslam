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

### 座標推定エラー通知
- Topic：/tagslam/pose_estimation_error
- Node：(prefix)_tagslam
- Type： std_msgs/msg/UInt8MultiArray
- Note：タグ未検出のみのフレームではエラーコードは追加しません。タグ未検出以外で座標推定に失敗した場合に、1 frame 分のエラーコードを `data` 配列にまとめて処理の最後に publish。エラーがない場合は空配列を publish します。前回 publish と内容が同じ場合は 10 frame に 1 回 publish
- Error Code：

| Code | Name | 詳細 | 解消方法 |
| --- | --- | --- | --- |
| 0 | NONE | エラーなし。通常 publish されません。 | 対応不要です。 |
| 1 | NO_INPUT | TagSLAM に処理対象のタグ検出メッセージが届いていません。 | detector / sync node が起動しているか、`cameras.<cam>.tag_topic` が正しいか、launch の namespace / remap が一致しているか確認してください。 |
| 2 | ALL_INPUTS_FILTERED | タグ検出メッセージは届きましたが、前処理の結果、利用できるタグが残りませんでした。 | 直前ログと `/tagslam/pose_estimation_warning` を確認し、`max_hamming_distance`、`amnesia`、タグ ID の登録状態を見直してください。 |
| 3 | OLD_TIMESTAMP | 受信 timestamp が前回処理済み timestamp 以下です。 | bag 再生順、シミュレーション clock、sync node の timestamp 付け替え、同じ topic への複数 publisher 混在を確認してください。 |
| 4 | TAG_MESSAGE_SIZE_MISMATCH | 設定されている camera 数と、TagSLAM に渡されたタグ検出メッセージの数が一致していません。 | `cameras.yaml` の camera 数、sync node の対象 camera 数、tagslam と sync node が同じ camera 設定ファイルを使っているか確認してください。 |
| 5 | OPTIMIZER_FAILED | pose graph の最適化処理で例外が発生しました。タグ位置、カメラパラメータ、観測値のどこかに大きな矛盾がある可能性があります。 | タグサイズ、タグの地図上pose、camera intrinsics / extrinsics、外れ値タグを確認してください。`error_map.txt` や直前ログに詳細が表示されます。 |
| 6 | BODY_POSE_NOT_OPTIMIZED | ロボット body の pose が得られませんでした。座標未定のタグしか観測できていない可能性が高いです。 | 原点タグや既知のタグが映るようにロボットを動かしてください。pose graph 系エラー `7-10` や warning が同時に出ていないかも確認してください。 |
| 7 | POSE_GRAPH_NO_NEW_FACTORS | 位置推定に使えるタグ入力がありませんでした。重複入力、設定による除外、または既に処理済みの観測だけだった可能性があります。 | timestamp 異常、warning が出ていないか、検出タグ ID が設定されているかを確認してください。 |
| 8 | POSE_GRAPH_NO_SUBGRAPH | 検出タグから、現在位置を推定するための有効な pose graph を作れませんでした。既知の地図・body・camera との接続が不足しています。 | 検出タグが既知タグとして登録されているか、camera pose が初期化されているか、`default_body` やタグ定義が現在の運用に合っているか確認してください。 |
| 9 | POSE_GRAPH_INITIALIZATION_FAILED | pose graph は作れましたが、最適化前に必要な初期 pose を決められませんでした。観測条件または拘束が不足しています。 | 同時に見えるタグ数を増やす、タグを大きく写す、既知タグ/既知camera poseを追加する、タグ配置と camera pose 設定を確認してください。 |
| 10 | POSE_GRAPH_ERROR_TOO_LARGE | pose graph 最大誤差が `max_subgraph_error` 以上だったため、姿勢推定に失敗しました。 | タグの配置が変わっていないか、tag sizeに誤りがないか、camera intrinsics / extrinsics が適切かどうかを確認してください。`max_subgraph_error` を緩める場合は姿勢推定精度が低くなる場合があります。 |

- 補足：
  - タグが単に未検出のフレームでは `data` にエラーコードは入りません。
  - 複数原因が同時に起きる場合、発生したエラーコードが `data` 配列に追加されます。
- Usage：
```
ros2 topic echo /tagslam/pose_estimation_error
```

### 座標推定ワーニング通知
- Topic：/tagslam/pose_estimation_warning
- Node：(prefix)_tagslam
- Type： triorb_slam_interface/msg/TagSlamWarning
- Note：座標推定に使えなかったタグ単位の警告です。`warning_codes[i]` と `tag_ids[i]` が対応します。warning がない場合は空配列を publish します。前回 publish と内容が同じ場合は 10 frame に 1 回 publish します。
- Message：
  - `uint8[] warning_codes`
  - `int32[] tag_ids`
- Warning Code：

| Code | Name | 詳細 | 解消方法 |
| --- | --- | --- | --- |
| 0 | NONE | warning code としては通常使用しません。warning がない場合は `warning_codes` / `tag_ids` が空配列の msg を publish します。 | 対応不要です。 |
| 1 | TAG_FILTERED_BY_HAMMING_DISTANCE | 該当タグの誤り訂正量 `hamming` が `max_hamming_distance` を超えたため、信頼できないタグとして除外されました。 | `tag_ids` のタグについて、画像のブレ、露光、ピント、タグ印刷品質、照明を改善してください。`max_hamming_distance` を変更する場合、誤検出増加に注意してください。 |
| 2 | TAG_FILTERED_BY_AMNESIA | `amnesia` 有効時に、現在の地図/設定に存在しない未知タグを検出したため除外されました。 | `tag_ids` のタグを地図または設定に登録してください。 |
| 3 | TAG_FILTERED_BY_BODY_CONFIG | 該当タグが body 設定上利用できません。`ignore_tags` 対象、`default_body` 未設定、default tag size 不正などが原因です。 | `tag_ids` のタグが body の `tags` に定義されているか、`ignore_tags` に含まれていないか、未知タグ利用時は `default_body` と default tag size が正しいか確認してください。 |
| 4 | TAG_FILTERED_BY_MINIMUM_AREA | 該当タグの画像面積が `minimum_tag_area` 未満のため除外されました。 | `tag_ids` のタグへ近づく、タグサイズを大きくする、カメラ解像度や焦点距離を見直すことで改善します。必要なら `minimum_tag_area` を下げてください。 |
| 5 | POSE_INIT_LOW_VIEWING_ANGLE | 該当タグを見る角度が `minimum_viewing_angle` 未満で、pose 初期値を安定して作れませんでした。 | `tag_ids` のタグをより正面から見る、タグ配置を変える、同時に見えるタグを増やすことで改善します。必要なら `minimum_viewing_angle` を緩和してください。 |
| 6 | POSE_INIT_AMBIGUITY | 該当タグを斜め方向から観測したため pose の表裏反転リスクが高く、`ambiguity_angle_threshold` / `max_ambiguity_ratio` 条件により除外されました。 | `tag_ids` のタグの視野角を改善する、複数タグを同時観測する、タグサイズ/配置を見直すことで対処してください。 |

- Usage：
```
ros2 topic echo /tagslam/pose_estimation_warning
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
