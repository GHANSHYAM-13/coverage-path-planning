operation Readme for coverage server

ros2 launch my_bot launch_sim.launch.py gui:=false world:=/home/jd/dev_ws/src/my_bot/worlds/Dock.world use_sim_time:=true 

ros2 launch my_coverage localization.launch.py   map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml use_sim_time:=true 

ros2 launch my_coverage coverage_nav.launch.py map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml use_sim_time:=true

ros2 launch my_coverage coverage_gui.launch.py   map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml