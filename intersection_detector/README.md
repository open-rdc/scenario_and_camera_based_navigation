# intersection_detector
単眼カメラ画像から，深層学習により通路の特徴を分類するパッケージ

## ネットワーク  
MobileNetV3-Large + lstm + fc

- 入力: カメラ画像（64×48）
- 出力: 通路の特徴 ( 8 種類)
0. straight_road
1. dead_end
2. corner_right
3. corner_left
4. cross_road
5. 3_way_right
6. 3_way_center
7. 3_way_left  

## Subscribed Topics
| Name                       | Type                                                                                         | Description                              |
| -------------------------- | -------------------------------------------------------------------------------------------- | ---------------------------------------- |
| `/camera_center/image_raw` | [`sensor_msgs/Image`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)     | center camera image                      |
| `/clock`                   | [`rosgraph_msgs/Clock`](http://docs.ros.org/en/noetic/api/rosgraph_msgs/html/msg/Clock.html) | simulation clock                         |
| `/cmd_dir_intersection`    | `scenario_navigation_msgs/cmd_dir_intersection` *(custom message)*                           | intersection command input ※learning only|

## Published Topics
| Name            | Type                                                                                     | Description                         |
| --------------- | ---------------------------------------------------------------------------------------- | ----------------------------------- |
| `/passage_type` | `scenario_navigation_msgs/cmd_dir_intersection` *(custom message)*                       | predicted passage/intersection type |

## パッケージの構成
```
├── CMakeLists.txt
├── README.md
├── data
│   └── model
│       └── demo
│           └── model.pt
├── launch
│   ├── intersection_detector.launch #通路分類を行うノードを実行
│   ├── intersection_detector_sim.launch #根幹となる gazebo や navigation を実行
│   └── start_wp_nav.launch #navigation, 学習を開始するサービスを投げる
├── package.xml
└── scripts
    ├── create_dataset_and_learning.py #データセット作成と学習を行うノード
    ├── network.py #深層学習器
    └── test.py #テストを行うノード
```

## 先行研究
https://ieeexplore.ieee.org/document/8206317  