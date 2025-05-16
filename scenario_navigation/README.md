# scenario_navigation  
人が道案内に用いる情報（シナリオ）を分解し,「条件」と「行動」を抽出するパッケージ

## パッケージの構成
```
├── CMakeLists.txt
├── LICENSE
├── README.md
├── config
│   └── Scenarios
│       ├── images_scenario01-50.pdf
│       ├── scenario01.txt
│       ├── scenario02.txt
│       ├── scenario03.txt
│       ├── scenario04.txt
#################### サンプルのシナリオ ###########################
│       └── scenario50.txt
├── launch
│   └── scenario_navigation.launch
├── msg
│   └── PassageType.msg
├── package.xml
├── scripts
│   └── scenario_parser_jsk.py
├── src
│   └── cmd_dir_executor_detailed.cpp
└── srv
    └── Scenario.srv
```




