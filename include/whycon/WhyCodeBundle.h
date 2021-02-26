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

#define NUM_FILTER_VALUES 30

using namespace std;
using namespace Eigen;

namespace whycon
{

class WhyCodeBundle
{
public:
	WhyCodeBundle(int _id, string _name);
	~WhyCodeBundle();

	bool process_bundle(const whycon::MarkerArray & markers, whycon::Bundle & bundle_out);
	int get_id();

	SBundleOrientationTracker orientation_tracker;

private:
	int id;
	string name;
	double angle;
	geometry_msgs::Pose pose;
	geometry_msgs::Vector3 rotation;		// order: roll, pitch, yaw
	geometry_msgs::Point camera_translation;	// order: up, west, north
	double u;
	double v;
	whycon::Bundle message;

	std::vector< Vector3d > points;
	std::vector< Vector3d > detected_marker_ids;
	std::pair<Eigen::Vector3d, Eigen::Vector3d> best_plane_from_points();
	void populate_message();

	// to do
	int queue_index = 0;
	geometry_msgs::Point past_camera_translation[NUM_FILTER_VALUES];
};

} // end namespace whycon
#endif
