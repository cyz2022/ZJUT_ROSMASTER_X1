roslaunch yahboomcar_nav laser_bringup.launch 

roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=cartographer

roslaunch yahboomcar_nav view_cartographer.launch

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

bash ~/yahboomcar_ws/src/yahboomcar_nav/maps/carto_map.sh  