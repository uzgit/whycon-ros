int8 id                         	# ID
int32 size                      	# in pixels
float32 u                       	# camera coordinates
float32 v                       	# camera coordinates
float32 angle                   	# rotation around surface normal
geometry_msgs/Pose  position    	# pose
geometry_msgs/Vector3 rotation  	# euler angles roll, pitch, yaw
geometry_msgs/Point camera_translation	# translational position of the camera in the coordinate frame of the marker (no rotation)
int8 solution_index			# the chosen solution for the WhyCode marker's orientation \in [0, 1]
