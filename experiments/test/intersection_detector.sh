for i in `seq 1`
do
  roslaunch intersection_detector intersection_detector_sim.launch script:=test.py world_name:=tsudanuma2-3.world map_file:=cit_3f_map waypoints_file:=route_a-f.yaml dist_err:=1.0 initial_pose_x:=-5.0 initial_pose_y:=7.7 initial_pose_a:=3.14 use_waypoint_nav:=false robot_x:=0.0 robot_y:=0.0 robot_Y:=0.0
  sleep 10
done
