#include "whycon/WhyCodeBundle.h"

#include <iostream>

namespace whycon
{

WhyCodeBundle::WhyCodeBundle(int _id, string _name)
{
	id = _id;
	name = _name;

	trans_ = new CTransformation(0.5);
}

WhyCodeBundle::~WhyCodeBundle()
{
	delete trans_;
}

bool WhyCodeBundle::process_bundle(std::vector<whycon::SMarker> markers, whycon::Bundle & bundle_out)
{
	bool result = false;

	if( markers.size() >= 3 )
	{
		angle = 0;
		points.clear();

		// get the points
		for(whycon::SMarker &detection : markers)
		{
			points.push_back( Eigen::Vector3d( detection.obj.x, detection.obj.y, detection.obj.z ) );
			angle += detection.obj.angle;

/*
			whycon::Marker marker;

			marker.id = detection.seg.ID;
			marker.size = detection.seg.size;
			marker.u = detection.seg.x;
			marker.v = detection.seg.y;
			marker.angle = detection.obj.angle;

			// Convert to ROS standard Coordinate System
			marker.position.position.x = detection.obj.x;
			marker.position.position.y = detection.obj.y;
			marker.position.position.z = detection.obj.z;
			marker.position.orientation.x = detection.obj.qx;
			marker.position.orientation.y = detection.obj.qy;
			marker.position.orientation.z = detection.obj.qz;
			marker.position.orientation.w = detection.obj.qw;
			marker.rotation.x = detection.obj.roll;
			marker.rotation.y = detection.obj.pitch;
			marker.rotation.z = detection.obj.yaw;
			marker.solution_index = detection.obj.segIdx;	

			printf("%d\n", detection.seg.ID);
			printf("%10.10f\t%10.10f\n", detection.obj.centers.n[0][0], detection.obj.centers.n[1][0]);
			printf("%10.10f\t%10.10f\n", detection.obj.centers.n[0][1], detection.obj.centers.n[1][1]);
			printf("%10.10f\t%10.10f\n", detection.obj.centers.n[0][2], detection.obj.centers.n[1][2]);
*/
		}
		
		// finish averaging the angle
		angle /= markers.size();

		// get the centroid and normal vector of the plane
		std::pair<Eigen::Vector3d, Eigen::Vector3d> result_pair = best_plane_from_points();
		Eigen::Vector3d centroid = result_pair.first;
		Eigen::Vector3d normal   = result_pair.second;

		// if the marker is facing away from us we can simply flip the orientation because we can assume the marker is actually facing us.
		if( normal[0] < 0 )
		{
			normal[0] *= -1;
			normal[1] *= -1;
			normal[2] *= -1;
		}

		// for now just consider the position of the bundle to be its centroid
		pose.position.x = centroid[0];
		pose.position.y = centroid[1];
		pose.position.z = centroid[2];

		// we consider the orientation quaternion to be the rotation from a unit "up" vector to the normal of the plane
		Eigen::Vector3d up = Eigen::Vector3d::UnitX();
		Eigen::Quaterniond initial_orientation = Eigen::Quaterniond::FromTwoVectors(up, normal);
		
	        double w = initial_orientation.w();
	        auto quaternion_axis = initial_orientation.vec();

		pose.orientation.w = w;
		pose.orientation.x = quaternion_axis[0];
		pose.orientation.y = quaternion_axis[1];
		pose.orientation.z = quaternion_axis[2];

		double roll, pitch, yaw;
		tf2::Quaternion orientation, orientation_inverse;
		tf2::fromMsg(pose.orientation, orientation);
		tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

		// to be in the whycon convention
		pitch = -pitch;

		orientation.setRPY(angle, pitch, yaw);

		rotation.x = roll;
		rotation.y = pitch;
		rotation.z = yaw;

		pose.orientation = tf2::toMsg(orientation);
#if 0
		double average_roll = 0;
		double average_pitch = 0;
		double average_yaw = 0;

		for(int i = 0; i < marker_array.markers.size(); i ++)
		{
			average_roll += marker_array.markers[i].rotation.x;
			average_pitch += marker_array.markers[i].rotation.y;
			average_yaw += marker_array.markers[i].rotation.z;
		}

		average_roll /= marker_array.markers.size();
		average_pitch /= marker_array.markers.size();
		average_yaw /= marker_array.markers.size();

		printf("%f\t", angle);
		printf("%f\t", average_roll);
		for(int i = 0; i < marker_array.markers.size(); i ++) printf("%f\t", marker_array.markers[i].rotation.x);
		printf("\n");
		printf("%f\t", pitch);
		printf("%f\t", average_pitch);
		for(int i = 0; i < marker_array.markers.size(); i ++) printf("%f\t", marker_array.markers[i].rotation.y);
		printf("\n");
		printf("%f\t", yaw);
		printf("%f\t", average_yaw);
		for(int i = 0; i < marker_array.markers.size(); i ++) printf("%f\t", marker_array.markers[i].rotation.z);
		printf("\n");
		printf("\n");
#endif

		orientation_inverse = orientation.inverse();
		geometry_msgs::TransformStamped rotation_transform;
		rotation_transform.transform.rotation = tf2::toMsg(orientation_inverse);
		
		geometry_msgs::Pose temp_pose;
		tf2::doTransform(pose, temp_pose, rotation_transform);
		camera_translation = temp_pose.position;

		populate_message();
		bundle_out = message;

/*
		for(whycon::SMarker &detection : markers)
		{
			double mynorm = sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]);
			double norm[2];
			norm[0] = sqrt(detection.obj.centers.n[0][0]*detection.obj.centers.n[0][0] + detection.obj.centers.n[0][1]*detection.obj.centers.n[0][1] + detection.obj.centers.n[0][2]*detection.obj.centers.n[0][2]);
			norm[1] = sqrt(detection.obj.centers.n[1][0]*detection.obj.centers.n[1][0] + detection.obj.centers.n[1][1]*detection.obj.centers.n[1][1] + detection.obj.centers.n[1][2]*detection.obj.centers.n[1][2]);

			double dot_product[2];
			dot_product[0] = normal[0]*detection.obj.centers.n[0][0] + normal[1]*detection.obj.centers.n[0][1] + normal[2]*detection.obj.centers.n[0][2];
			dot_product[1] = normal[0]*detection.obj.centers.n[1][0] + normal[1]*detection.obj.centers.n[1][1] + normal[2]*detection.obj.centers.n[1][2];
			
			printf("%d\n", detection.seg.ID);
			printf("%10.10f\t%10.10f\t%10.10f\n", norm[0], norm[1], mynorm);
			printf("%10.10f\t%10.10f\t%10.10f\n", detection.obj.centers.n[0][0], detection.obj.centers.n[1][0], normal[0]);
			printf("%10.10f\t%10.10f\t%10.10f\n", detection.obj.centers.n[0][1], detection.obj.centers.n[1][1], normal[1]);
			printf("%10.10f\t%10.10f\t%10.10f\n", detection.obj.centers.n[0][2], detection.obj.centers.n[1][2], normal[2]);
			printf("%10.10f\t%10.10f\n", dot_product[0], dot_product[1]);

			
		}
*/		

		for(whycon::SMarker &detection : markers)
		{
			double dist_x;
			double dist_y;
			double dist_z;
			double dist[2];
			STrackedObject buffer;
			geometry_msgs::Point buffer2[2];
			for(int i = 0; i < 2; i ++)
			{
				buffer.u = detection.obj.centers.u[i];
				buffer.v = detection.obj.centers.v[i];
				buffer.x = detection.obj.centers.t[i][0];
				buffer.y = detection.obj.centers.t[i][1];
				buffer.z = detection.obj.centers.t[i][2];
				buffer.n0 = detection.obj.centers.n[i][0];
				buffer.n1 = detection.obj.centers.n[i][1];
				buffer.n2 = detection.obj.centers.n[i][2];
				
				trans_->calcOrientation(buffer);
				trans_->transformCoordinates(buffer);

				geometry_msgs::Pose pose;
				pose.position.x = buffer.x;
				pose.position.y = buffer.y;
				pose.position.z = buffer.z;

				pose.orientation.x = buffer.qx;
				pose.orientation.y = buffer.qy;
				pose.orientation.z = buffer.qz;
				pose.orientation.w = buffer.qw;

				geometry_msgs::Point temp = get_camera_translation(pose);
				buffer2[i] = temp;

				dist_x = camera_translation.x - temp.x;
				dist_y = camera_translation.y - temp.y;
				dist_z = camera_translation.z - temp.z;

				dist[i] = sqrt( dist_x*dist_x + dist_y*dist_y + dist_z*dist_z );
				
			}

			std::cout << std::endl << std::endl;

			int segIdx = -1;
//			bool decision_metric = dot_product[0] > dot_product[1];
			if( dist[0] < dist[1] )
			{
				segIdx = 0;
			}
			else
			{
				segIdx = 1;
			}

			// regenerate the orientation and transform if we think the segIdx was wrong
//			if( segIdx != detection.obj.segIdx && abs(decision_metric) > 0.01 )
			if( segIdx != detection.obj.segIdx )
			{
				detection.obj.u = detection.obj.centers.u[segIdx];
				detection.obj.v = detection.obj.centers.v[segIdx];
				detection.obj.x = detection.obj.centers.t[segIdx][0];
				detection.obj.y = detection.obj.centers.t[segIdx][1];
				detection.obj.z = detection.obj.centers.t[segIdx][2];
				detection.obj.n0 = detection.obj.centers.n[segIdx][0];
				detection.obj.n1 = detection.obj.centers.n[segIdx][1];
				detection.obj.n2 = detection.obj.centers.n[segIdx][2];
//				detection.obj.segIdx = segIdx;

				trans_->calcOrientation(detection.obj);
				trans_->transformCoordinates(detection.obj);

//				std::cout << "JRKSLJFKLSDFJKLSDFJSDKLFSD" << std::endl;
			}

			std::cout << buffer2[segIdx] << std::endl;

//			printf("%d\t%d\n", detection.obj.segIdx, segIdx);
//			printf("\n");

		}	

		result = true;
	}
	return result;
}

bool WhyCodeBundle::process_bundle(const whycon::MarkerArray & marker_array, whycon::Bundle & bundle_out)
{
	bool result = false;

	// we need at least 3 points to make a plane
	// need to add another check for non-colinearity 
	if( marker_array.markers.size() >= 3 )
	{
		// initialize the perceived yaw angle
		angle = 0;

		// clear the points vector
		points.clear();

		// get all the points in the correct format and average the angle
		for(whycon::Marker marker : marker_array.markers)
		{
			points.push_back( Eigen::Vector3d( marker.position.position.x, marker.position.position.y, marker.position.position.z ) );
			angle += marker.angle;
		}

		// finish averaging the angle
		angle /= marker_array.markers.size();

		// get the centroid and normal vector of the plane
		std::pair<Eigen::Vector3d, Eigen::Vector3d> result_pair = best_plane_from_points();
		Eigen::Vector3d centroid = result_pair.first;
		Eigen::Vector3d normal   = result_pair.second;

		// if the marker is facing away from us we can simply flip the orientation because we can assume the marker is actually facing us.
		if( normal[0] < 0 )
		{
			normal[0] *= -1;
			normal[1] *= -1;
			normal[2] *= -1;
		}

		double dot_product[2];

		for(int i = 0; i < 3; i ++)
		{
			orientation_tracker.n[i] = normal[i];
		}

		// for now just consider the position of the bundle to be its centroid
		pose.position.x = centroid[0];
		pose.position.y = centroid[1];
		pose.position.z = centroid[2];

		// we consider the orientation quaternion to be the rotation from a unit "up" vector to the normal of the plane
		Eigen::Vector3d up = Eigen::Vector3d::UnitX();
		Eigen::Quaterniond initial_orientation = Eigen::Quaterniond::FromTwoVectors(up, normal);
		
	        double w = initial_orientation.w();
	        auto quaternion_axis = initial_orientation.vec();

		pose.orientation.w = w;
		pose.orientation.x = quaternion_axis[0];
		pose.orientation.y = quaternion_axis[1];
		pose.orientation.z = quaternion_axis[2];

		double roll, pitch, yaw;
		tf2::Quaternion orientation, orientation_inverse;
		tf2::fromMsg(pose.orientation, orientation);
		tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

		// to be in the whycon convention
//		pitch = -pitch;

		orientation.setRPY(angle, pitch, yaw);

		rotation.x = roll;
		rotation.y = pitch;
		rotation.z = yaw;

		pose.orientation = tf2::toMsg(orientation);
#if 0
		double average_roll = 0;
		double average_pitch = 0;
		double average_yaw = 0;

		for(int i = 0; i < marker_array.markers.size(); i ++)
		{
			average_roll += marker_array.markers[i].rotation.x;
			average_pitch += marker_array.markers[i].rotation.y;
			average_yaw += marker_array.markers[i].rotation.z;
		}

		average_roll /= marker_array.markers.size();
		average_pitch /= marker_array.markers.size();
		average_yaw /= marker_array.markers.size();

		printf("%f\t", angle);
		printf("%f\t", average_roll);
		for(int i = 0; i < marker_array.markers.size(); i ++) printf("%f\t", marker_array.markers[i].rotation.x);
		printf("\n");
		printf("%f\t", pitch);
		printf("%f\t", average_pitch);
		for(int i = 0; i < marker_array.markers.size(); i ++) printf("%f\t", marker_array.markers[i].rotation.y);
		printf("\n");
		printf("%f\t", yaw);
		printf("%f\t", average_yaw);
		for(int i = 0; i < marker_array.markers.size(); i ++) printf("%f\t", marker_array.markers[i].rotation.z);
		printf("\n");
		printf("\n");
#endif

		orientation_inverse = orientation.inverse();
		geometry_msgs::TransformStamped rotation_transform;
		rotation_transform.transform.rotation = tf2::toMsg(orientation_inverse);
		
		geometry_msgs::Pose temp_pose;
		tf2::doTransform(pose, temp_pose, rotation_transform);
		camera_translation = temp_pose.position;

		populate_message();
		bundle_out = message;

		result = true;
	}
	return result;
}

std::pair<Vector3d, Vector3d> WhyCodeBundle::best_plane_from_points()
{
        // copy coordinates to  matrix in Eigen format
        size_t num_atoms = points.size();
        Matrix< Vector3d::Scalar, Dynamic, Dynamic > coord(3, num_atoms);
        for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = points[i];

        // calculate centroid
        Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

        // subtract centroid
        coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

        // we only need the left-singular matrix here
        //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
        auto svd = coord.jacobiSvd(ComputeThinU | ComputeThinV);
        auto plane_normal = svd.matrixU().rightCols(1);
        return std::make_pair(centroid, plane_normal);
}

geometry_msgs::Point WhyCodeBundle::get_camera_translation( const geometry_msgs::Pose & marker_pose )
{
        // the pose of the camera within the marker's coordinate frame
        geometry_msgs::Pose camera_pose;

        // get the marker's orientation and invert it
        tf2::Quaternion marker_orientation, marker_orientation_inverse;
        tf2::fromMsg(marker_pose.orientation, marker_orientation);
        marker_orientation_inverse = marker_orientation.inverse();

        // create a rotational transform with the inverse of the marker's orientation (no translational element)
        geometry_msgs::TransformStamped rotation_transform;
        rotation_transform.transform.rotation = tf2::toMsg(marker_orientation_inverse);

        // carry out the rotation
        tf2::doTransform(marker_pose, camera_pose, rotation_transform);

        // return only the translational elements of the camera's pose, as the orientation will be (w, x, y, z) ≈ (1, 0, 0, 0)
        return camera_pose.position;
}

int WhyCodeBundle::get_id()
{
	return id;
}

void WhyCodeBundle::populate_message()
{
#if 0
	if( queue_index == NUM_FILTER_VALUES )
	{
		queue_index = 0;
	}
	
	past_camera_translation[queue_index] = camera_translation;
	geometry_msgs::Point temp;
	temp.x = 0;
	temp.y = 0;
	temp.z = 0;
	for(int i = 0; i < NUM_FILTER_VALUES; i ++)
	{
		temp.x += past_camera_translation[i].x;
		temp.y += past_camera_translation[i].y;
		temp.z += past_camera_translation[i].z;
	}
	temp.x /= NUM_FILTER_VALUES;
	temp.y /= NUM_FILTER_VALUES;
	temp.z /= NUM_FILTER_VALUES;
	
	message.camera_translation_uwn = temp;

	queue_index ++;
#else
	message.camera_translation_uwn = camera_translation;

#endif

	message.ID = id;
	message.name = "";
	message.position = pose;
	message.rotation_rpy = rotation;
	message.u = u;
	message.v = v;
}

} // end namespace whycon
