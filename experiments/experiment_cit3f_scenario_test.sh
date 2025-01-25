for i in `seq 1`
do
  roslaunch nav_cloning nav_cloning_sim.launch script:=nav_cloning_with_direction_intersection_node_test.py mode:=selected_training world_name:=tsudanuma2-3_v2.3.3.world map_file:=cit_3f_map.yaml waypoints_file:=cit3f_way_intersection.yaml dist_err:=1.0 initial_pose_x:=-5.0 initial_pose_y:=7.7 initial_pose_a:=3.14 use_waypoint_nav:=false robot_x:=-5.0 robot_y:=7.7 robot_Y:=3.14
  sleep 10
done