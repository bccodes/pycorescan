ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ./config1.yaml --remap __ns:=/usb_cam_0;
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ./config2.yaml --remap __ns:=/usb_cam_1;

ros2 run foxglove_bridge foxglove_bridge;

# to save images, first
ros2 run image_view image_saver --ros-args -r image:=/usb_cam_0/image_raw  -p save_all_image:=false;
# then run
# ros2 service call /save

# to set params
ros2 run demo_nodes_cpp parameter_blackboard
# then
# ros2 param set /parameter_blackboard/some_param
