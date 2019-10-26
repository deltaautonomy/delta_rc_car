# rosbag files
To record ros file - `rosrecord`
To run robag file - `rosbag play -l yolo_train_2.bag`
To view the rosbag file - `rosrun image_view image_view image:=/pt_grey/image_color_rect`
To extract images from rosbag - follow - http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

# label creation 
yolo_mark

# Running trained model
`./darknet detector test data/obj.data  cfg/yolo-rc_car.cfg backup/yolo-rc_car_2000.weights data_test/frame0473.jpg`
(cfg data and names files are stored in yolo_train_setup folder)
