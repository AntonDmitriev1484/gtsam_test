#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"

#include "cmath"
#include <regex>

using namespace gtsam;
using namespace std;

#define PLOT(GT_TRAJECTORY, GT_POINTS, EST_TRAJECTORY, VIO_TRAJECTORY) {			   \
                auto fig = figure();                                       \
                fig->name("Trajectory");                      \
				title("Trajectories");                                          \
                                                                           \
                hold(on);                                                  \
                draw_trajectory(VIO_TRAJECTORY, "red");               \
                hold(on);                                                  \
				draw_points(GT_POINTS, "green");							\
				hold(on);													\
                draw_trajectory(GT_TRAJECTORY, "green");                      \
                hold(on);                                                  \
                draw_trajectory(EST_TRAJECTORY, "blue");              \
                                                                           \
                xlabel("X (m)");                                           \
                ylabel("Y (m)");                                           \
                zlabel("Z (m)");                                           \
                                                                           \
				zlim({0,4});												\
}

void mini_uwb_static_anchors() {
	freopen("/dev/null", "w", stderr); // To mute Matplot++ errors in output stream

	// Make Key lambda
	const function<Key(string, int)> MK = [](string username, int I) {
		Key k;
		if (username.find("static") != std::string::npos) {
			regex numberRegex(R"(\d+$)");
			smatch match;
			regex_search(username, match, numberRegex);
			k = symbol('s', stoi(match.str())); // e.x. s11 if 'static11'
		}
		else {
			k = symbol(username[0], I);
			if (username == "jeff") k = symbol('f', I);
		}
		return k;
	};
	NonlinearFactorGraph* graph = new NonlinearFactorGraph();

	// Define noise models:

	// GT Noise model
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.0174533;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));

	// VIO noise model
	double vio_ori_stdev = 0.1; // 5.7deg
	double vio_pos_stdev = 0.05; // 5cm
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model
	double uwb_stdev = 0.01;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);




	// -> Generate GT trajectory
	vector<Pose3> true_trajectory;
	Point3 initial_pos(0, 0, 2);

	Rot3 initial_rot = Rot3::Identity(); // along the +x axis
	Pose3 initial_pose(initial_rot, initial_pos);
	true_trajectory.push_back(initial_pose);

	double theta = 0.125; // Rotation about the z-axis by some radians
	Matrix3 d_rot_mat;
	d_rot_mat << cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1;

	Pose3 d_pose(Rot3(d_rot_mat), Point3(1, 0, 0));

	int N_poses = 25;
	for (int i = 0; i < N_poses; i++) {
		true_trajectory.push_back(d_pose * true_trajectory.back());
	}

	// Discrete GT points
	vector<Pose3> gt_points;
	for (int i = 0; i < N_poses; i += 3) {
		gt_points.push_back(true_trajectory[i]);
	}

	// -> Generate drifted VIO trajectory
	vector<Pose3> vio_trajectory;
	Pose3 offset_vio(Rot3::Identity(), Point3(0, 0, 0)); // Assume no offset from initial GT pose
	vio_trajectory.push_back(offset_vio * initial_pose);


	graph->addPrior<Pose3>(MK("x", 0), vio_trajectory[0], GT_noise_model); 
	Values vals;
	vals.insert(MK("x", 0), vio_trajectory[0]);

	double theta_drift = -0.04;
	Matrix3 drift_rot_mat;
	drift_rot_mat << cos(theta_drift), -sin(theta_drift), 0,
		sin(theta_drift), cos(theta_drift), 0,
		0, 0, 1;
	Rot3 drift_rot(drift_rot_mat);
	Pose3 drift_vio(drift_rot, Point3(+0.0, +0.02, 0));

	for (int i = 1; i < N_poses; i++) {
		Pose3 vio_pose = (d_pose * drift_vio) * vio_trajectory.back();
		vio_trajectory.push_back(vio_pose);
	}


	// Generate UWB Anchor location(s)
	//vector<Point3> anchors = { Point3(-10, 0, 0), Point3(0,10,0), Point3(10,0,0), Point3(0, -10 , 20)};
	vector<Point3> anchors = { Point3(0,0,0) };

	for (int i = 0; i < anchors.size(); i++) {
		vals.insert(MK("a", i), anchors[i]);
		// Per suggestion at: https://groups.google.com/g/gtsam-users/c/vgczSzeYdoM/m/d7_b_WzYAQAJ
		graph->add(NonlinearEquality<Point3>(MK("a", i), anchors[i]));
		//graph->addPrior<Point3>(MK("a", i), anchors[i], noiseModel::Diagonal::Sigmas(Vector3(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev)));
	}

	// Main loop: Building the graph !
	for (int i = 1; i < N_poses; i++) {
		// Add odometry factor
		vals.insert(MK("x", i), vio_trajectory[i]);
		Pose3 odometry = vio_trajectory.back().between(vio_trajectory[i]);
		graph->add(BetweenFactor<Pose3>(MK("x", i - 1), MK("x", i), odometry, VIO_pose_noise_model));

		if (i % 1 == 0) {
			// Add UWB ranging factor
			for (int j = 0; j < anchors.size(); j++) {
				double true_distance = distance3(true_trajectory[i].translation(), anchors[j]);
				graph->add(RangeFactor<Pose3, Point3>(MK("x", i), MK("a", j), true_distance, UWB_noise_model));
			}
		}

	}


	//LevenbergMarquardtParams params;
	//LevenbergMarquardtOptimizer optimizer(*graph, vals, params);

	GaussNewtonParams params;
	GaussNewtonOptimizer optimizer(*graph, vals, params);

	double last_error;
	do {
		last_error = optimizer.error();
		optimizer.iterate();

		Values v = optimizer.values();

		vector<Pose3> est_trajectory;
		for (int i = 0; i < N_poses; i++) est_trajectory.push_back(v.at<Pose3>(MK("x", i)));
		PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);

		Marginals marg(*graph, v);

		cout << "------------" << endl;
		for (int i = 0; i < N_poses; i++) {
			if ( i == 10) cout << "x" << i << " \n" << marg.marginalCovariance(MK("x", i)) << ", ";
		}
		cout << endl;

		cout << "Error " << optimizer.error() << endl;

	} while (!checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol, last_error, optimizer.error()));

	cout << " Converged LM in " << optimizer.iterations() << " iterations, with " << optimizer.error() << " final error." << endl;


	Values final_vals = optimizer.values();
	vector<Pose3> est_trajectory;
	for (int i = 0; i < N_poses; i++) est_trajectory.push_back(final_vals.at<Pose3>(MK("x", i)));
	PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);

	show();

	graph->print();
}


int main(int argc, char* argv[]) {

	mini_uwb_static_anchors();

	return 0;
}
