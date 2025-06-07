# nav_cloning
深層学習により，カメラ画像と目標方向から，ロボットの角速度を出力するパッケージ

## ネットワーク
<img src="../data/source/nav_cloning/network.png">

- **入力**
  - カメラ画像: 64×48
  - 目標方向: 直進，左折，右折

- **出力**
  - ロボットの角速度: rad/s

## Subscribed Topics
| Name        | Type                                    | Description                       | 
| :---------- | :-------------------------------------- | :-------------------------------- | 
| `/amcl_pose`                | [`geometry_msgs/PoseWithCovarianceStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) | robot's current estimated pose           |
| `/camera_center/image_raw` | [`sensor_msgs/Image`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imagehtml)                                             | center camera image                      |
| `/camera_left/image_raw`   | [`sensor_msgs/Image`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)                                             | left camera image                        |
| `/camera_right/image_raw`  | [`sensor_msgs/Image`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)                                             | right camera image                       |
| `/clock`                   | [`rosgraph_msgs/Clock`](http://docs.ros.org/en/noetic/api/rosgraph_msgs/html/msg/Clock.html)                                         | simulation clock                         |
| `/cmd_dir_intersection`    | `scenario_navigation_msgs/cmd_dir_intersection` *(custom message)*                                                                   | direction and intersection command input |
| `/move_base/NavfnROS/plan` | [`nav_msgs/Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)                                                     | global planned path                      |
| `/nav_vel`                 | [`geometry_msgs/Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)                                         | velocity command from navigation stack   |
| `/tracker`                 | [`nav_msgs/Odometry`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)                                             | current odometry from tracker            |
※ Subscribed to `/camera_center` and `/cmd_dir_intersection` during the test.

## Published Topics

| Name       | Type                                                                                         | Description                           |
| ---------- | -------------------------------------------------------------------------------------------- | ------------------------------------- |
| `/cmd_vel` | [`geometry_msgs/Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) | velocity command output to the robot  |

 
## 構成
```
├── CMakeLists.txt
├── README.md
├── data # （dataset, result は初期状態では存在しない）
│   ├── dataset # カメラ画像と目標方向，目標角速度から構成されるデータセット
│   │   ├── dir # 目標方向
│   │   ├── image　# カメラ画像
│   │   └── vel # 目標角速度
│   ├── model # 学習機のモデル
│   │   └── demo
│   │       └── model.pt
│   └── result # 走行時のデータ
├── launch
│   ├── nav_cloning.launch # nav_cloning のノードを実行
│   ├── nav_cloning_sim.launch # 根幹となる gazebo や navigation を実行
│   └── start_wp_nav.launch # navigation, 学習を開始するサービスを投げる
├── network.png
├── package.xml
└── scripts
    ├── learning_node.py # 学習，テストを一括で行うノード
    ├── network.py # 深層学習機
    └── test_node.py # テストのみを行うノード
```