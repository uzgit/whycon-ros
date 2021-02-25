int8 ID						# the ID of the bundle
string name					# the name of the bundle
geometry_msgs/Pose position			# pose
geometry_msgs/Vector3 rotation_rpy		# euler angles roll, pitch, yaw
geometry_msgs/Point camera_translation_uwn	# translational position of the camera in the coordinate frame of the marker (no rotation) (order: up, west, north)
int8[] IDs					# the IDs of the detected markers that belong to the bundle
float32 u					# location of the x coordinate of the bundle center in camera coordinates
float32 v					# location of the y coordinate of the bundle center in camera coordinates
