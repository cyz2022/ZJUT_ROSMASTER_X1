roslaunch yahboomcar_nav astrapro_bringup.launch

roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping

roslaunch yahboomcar_nav view_vision_mapping.launch