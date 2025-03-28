# RPLidar Wall Filter
Project for the ARS vip. This node listens to LaserScan messages from /scan, transforms into 2d points about a robot's pose, trims all points that lie outside of a box, and publishes the new ranges back as a LaserScan.		

Simulate a transform message:
 ros2 run tf2_ros static_transform_publisher x y z 0 0 0 target_frame source_frame

Still need to:		
* Configure for point cloud
