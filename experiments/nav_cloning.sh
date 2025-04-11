for i in `seq 1`
do
  roslaunch nav_cloning nav_cloning_real.launch script:=learning_node.py map_file:=cit_3f_map waypoints_file:=route_a-f.yaml dist_err:=1.0 initial_pose_x:=-5.0 initial_pose_y:=7.7 initial_pose_a:=3.14 use_waypoint_nav:=true
  sleep 10
done
