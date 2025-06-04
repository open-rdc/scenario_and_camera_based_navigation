for i in `seq 1`
do
  roslaunch nav_cloning nav_cloning_sim.launch script:=learning_node.py mode:=selected_training world_name:=tsudanuma2-3.world map_file:=cit_3f_map waypoints_file:=route_a-f.yaml dist_err:=0.8 use_waypoint_nav:=true robot_x:=0.0 robot_y:=0.0 robot_Y:=0.0 gui:=false
  sleep 10
done
