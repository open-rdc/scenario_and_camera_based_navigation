# waypoint_nav
ウェイポイントに基づくナビゲーションを行いながら，データセットに通路の特徴の分類と目標方向を与えるパッケージ

## Settings
### ROS Param
For detail, see [run.launch](https://github.com/tiger0421/waypoint_nav/blob/main/launch/run.launch)
- robot_frame  
  ex) base_link

- world_frame  
  ex) map

- max_update_rate  
  ex) 1.0

- filename  
  Set your path to yaml

- dist_err  
  Regarding your robot as arriving at the target waypoint

- loop_flg  
  If this flg is true, do all over again after arriving at last waypoint

- wait_time  
  This is a time which your robot waits for moving  
  Send a waypoint again, if your robot doesn't move for this time

- resend_thresh  
  This is a threshold which how many times to resend the waypoint  
  Skip the waypoint if this threshold is over

### Yaml Setting
```
waypoints:
  - point:
    x:
    y:
    z:
    function:
```
You can set some functions if you add function to source program.  
This program has two functions, "run" and "suspend".  
However, "function" is not compulsory.  
See this [sample.yaml](https://github.com/tiger0421/waypoint_nav/blob/main/config/sample.yaml)

- run  
  This is normal mode.  
  When your robot gets at target waypoint, makes for next waypoint.
  Publish target direction:continue
- suspend  
  This makes your robot suspend mode after moving.
- run_go
  Publish target direction:go_straight
- run_left
  Publish target direction:go_left
- run_right
  Publish target direction:go_right

## Suspend Mode
When you want a robot to start or resume navigation, call `rosservice` like below.  
`
$ rosservice call /start_wp_nav "{}"
`

When you want a robot to set suspend mode, call `rosservice` like below.  
`
$ rosservice call /suspend_wp_nav "{}"
`
