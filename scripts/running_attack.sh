
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4 attack.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun joy joy_node; exec bash"' \
--tab -e 'bash -c "sleep 7; rosrun proportional_navigation_node proportional_navigation_node; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun lighttrack_ros LightTrackNode; exec bash"' \

