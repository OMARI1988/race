
$roscore
$rosparam set use_sim_time true
$ROS_NAMESPACE=/head_mount_kinect/rgb rosrun image_proc image_proc
$roslaunch openni_launch openni.launch load_driver:=false camera:=head_mount_kinect
$rosbag play --clock rosbag_file_name.bag
$rosrun race extracting_data.py
