# RPLidar Wall Filter
Project for the ARS vip. This node listens to LaserScan messages from /scan, transforms into 2d points about a robot's pose, trims all points that lie outside of a box, and publishes the new ranges back as a LaserScan.		

Still need to:		
* Test with the robot's actual pose (hardcoded pose and rotation for now)		
* Get the bounding box from a topic (hardcoded for now)
* Swap the message it published to a LaserScan, currently PolygonStamped for visualiziation
