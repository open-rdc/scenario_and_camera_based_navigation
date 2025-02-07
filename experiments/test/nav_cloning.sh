for i in `seq 1`
do
  roslaunch nav_cloning nav_cloning.launch script:=test_node.py
  sleep 10
done
