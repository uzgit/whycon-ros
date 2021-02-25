#include "whycon/WhyCodeBundle.h"

#include <iostream>

namespace whycon
{

WhyCodeBundle::WhyCodeBundle(int _id, string _name)
{
	id = _id;
	name = _name;
}

WhyCodeBundle::~WhyCodeBundle()
{
	
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
		orientation.setRPY(angle, pitch, yaw);

		rotation.x = roll;
		rotation.y = pitch;
		rotation.z = yaw;
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

int WhyCodeBundle::get_id()
{
	return id;
}

void WhyCodeBundle::populate_message()
{
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

	queue_index ++;

	message.ID = id;
	message.name = "";
	message.position = pose;
	message.rotation_rpy = rotation;
//	message.camera_translation_uwn = camera_translation;
	message.camera_translation_uwn = temp;
	message.u = u;
	message.v = v;
}

} // end namespace whycon
