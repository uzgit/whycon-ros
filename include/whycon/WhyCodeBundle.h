#ifndef WHYCODE_BUNDLE_H
#define WHYCODE_BUNDLE_H

#include "whycon/whycon.h"
#include "whycon/Marker.h"
#include "whycon/MarkerArray.h"
#include "whycon/Bundle.h"
#include "whycon/BundleArray.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace Eigen;

namespace whycon
{

class WhyCodeBundle
{
public:
	WhyCodeBundle(int bundle_id);
	~WhyCodeBundle();

//	template<class Vector3>
//	std::pair<Vector3, Vector3> WhyCodeBundle::best_plane_from_points(const std::vector<Vector3> & c);
//	std::pair<Eigen::Matrix<double, 3, 1, 0, 3, 1>, Eigen::Matrix<double, 3, 1, 0, 3, 1> > best_plane_from_points(const std::vector< Eigen::Matrix<double, 3, 1, 0, 3, 1> > & c);

	bool process_bundle(const whycon::MarkerArray & markers, geometry_msgs::Pose & pose_out);
	int get_id();

private:
	int id;
	std::vector< Eigen::Vector3d > points;
	std::vector< Eigen::Vector3d > detected_marker_ids;
	std::pair<Eigen::Vector3d, Eigen::Vector3d> best_plane_from_points();

	double angle;
	geometry_msgs::Pose pose;
	geometry_msgs::Vector3 rotation;
	geometry_msgs::Point camera_translation;
	double u;
	double v;
};

} // end namespace whycon
#endif
