for i in `seq 1`
do
  roslaunch nav_cloning nav_cloning_real.launch script:=learning_node.py map_file:=tsudanuma dist_err:=1.2 waypoints_file:=tsudanuma.yaml use_waypoint_nav:=true
  sleep 10
done