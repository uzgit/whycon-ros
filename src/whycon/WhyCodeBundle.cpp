#include "whycon/WhyCodeBundle.h"

#include <iostream>

namespace whycon
{

WhyCodeBundle::WhyCodeBundle(int bundle_id)
{
	id = bundle_id;
}

WhyCodeBundle::~WhyCodeBundle()
{
	
}

bool WhyCodeBundle::process_bundle(const whycon::MarkerArray & marker_array, geometry_msgs::Pose & pose_out)
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

		orientation_inverse = orientation.inverse();
		geometry_msgs::TransformStamped rotation_transform;
		rotation_transform.transform.rotation = tf2::toMsg(orientation_inverse);
		
		geometry_msgs::Pose temp_pose;
		tf2::doTransform(pose, temp_pose, rotation_transform);
		camera_translation = temp_pose.position;
		std::cout << camera_translation << std::endl;		

		// for debugging only
//		tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
//		printf("(%10.10f, %10.10f, %10.10f)\n", roll, pitch, yaw);

		// no consideration of yaw
//		Eigen::Vector3d initial_rotation = Eigen::Matrix3d(initial_orientation).eulerAngles(0, 1, 2);

/*
		double yaw   = angle;
		double pitch = initial_rotation[1];
		double roll  = initial_rotation[2];

		orientation.setRPY(roll, pitch, yaw);
		pose.orientation = tf2::toMsg(orientation);

//		std::cout << pose << std::endl;
		
		orientation_inverse = orientation.inverse();
		geometry_msgs::TransformStamped rotation_transform;
		rotation_transform.transform.rotation = tf2::toMsg(orientation_inverse);
		
		geometry_msgs::Pose temp_pose;
		tf2::doTransform(pose, temp_pose, rotation_transform);
		camera_translation = temp_pose.position;
		std::cout << camera_translation << std::endl;		
*/		
/*
		std::cout << initial_rotation << std::endl;



		tf2::Quaternion orientation, orientation_inverse;
		orientation.setRPY(roll, pitch, yaw);


		pose.orientation = tf2::toMsg(orientation);

		orientation_inverse = orientation.inverse();

		geometry_msgs::TransformStamped rotation_transform;
		rotation_transform.transform.rotation = tf2::toMsg(orientation_inverse);

		geometry_msgs::Pose temp_pose;
		tf2::doTransform(pose, temp_pose, rotation_transform);

		camera_translation = temp_pose.position;
*/
//		printf("(%10.10f, %10.10f, %10.10f)\n", rotation[0], rotation[1], rotation[2]);
/*
		tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
		printf("RPY: (%10.10f, %10.10f, %10.10f)\n", roll, pitch, yaw);
*/
//		printf("%f", w);


//		std::cout << camera_translation << std::endl;

		pose_out = pose;

		result = true;
	}
	return result;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> WhyCodeBundle::best_plane_from_points()
{
        // copy coordinates to  matrix in Eigen format
//        size_t num_atoms = c.size();
        size_t num_atoms = points.size();
        Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
//        for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];
        for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = points[i];

        // calculate centroid
        Eigen::Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());
        //Vector3 centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

        // subtract centroid
        coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

        // we only need the left-singular matrix here
        //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
        auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        //Vector3 plane_normal = svd.matrixU().rightCols<1>();
        auto plane_normal = svd.matrixU().rightCols(1);
        return std::make_pair(centroid, plane_normal);
}

int WhyCodeBundle::get_id()
{
	return id;
}

} // end namespace whycon
