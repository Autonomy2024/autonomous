
script_dir=$(dirname "$(readlink -f "$0")")


rio_setup_file="$script_dir/../../devel/setup.bash"

source $rio_setup_file

echo "start radar simulation"
roslaunch radar_simulation launch.launch & 

echo "start rio"
roslaunch ekf_rio demo_datasets_ekf-rio_ros.launch 

