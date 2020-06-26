source /opt/ros/noetic/setup.bash

sleep 2s

cd /exercises/amazon_warehouse/launch && roslaunch amazonrobot_1_warehouse.launch map_file:=/exercises/amazon_warehouse/launch/maps/map.yaml &

cd /exercises/amazon_warehouse/ && python3 amazonWarehouse.py amazonMap.conf amazonConf.yml