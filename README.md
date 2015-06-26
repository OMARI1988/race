
$roscore
$rosparam set use_sim_time true
$ROS_NAMESPACE=/head_mount_kinect/rgb rosrun image_proc image_proc
$roslaunch openni_launch openni.launch load_driver:=false camera:=head_mount_kinect
$rosrun race extracting_data.py
$rosbag play --clock -r .1 rosbag_file_name.bag
