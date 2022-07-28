The gpu_node might run surf (who knows?), if so it ran with opencv 3.4.0

Test different images:
Remember to change image path for your image in the node.


How to run (different terminals):

roscore

rosbag play ~/cv_ws/src/Vortex-CV/sift_feature_detection/data/pure_image.bag -l --clock -s85

rosrun image_transport republish compressed in:=/zed2/zed_node/rgb/image_rect_color  raw out:=/zed2/zed_node/rgb/image_rect_color

rosrun sift_feature_detection sift_node.py 

