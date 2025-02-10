#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"

#include "cmath"
#include <regex>

using namespace gtsam;
using namespace std;

#define PLOT_FOR_USERS(INFO, SHOW_LIST) {						           \
    for (const auto& [user_name, user_info] : INFO) {                      \
        if (!user_info.is_beacon) {                                        \
            if (find(SHOW_LIST.begin(), SHOW_LIST.end(), user_name) != SHOW_LIST.end()) { \
                auto fig = figure();                                       \
                fig->name(user_name + " trajectory");                      \
                title(user_name);                                          \
                                                                           \
                hold(on);                                                  \
                draw_trajectory(user_info.vio_poses, "red");               \
                hold(on);                                                  \
                draw_points(user_info.gt_poses, "g");                      \
                hold(on);                                                  \
                draw_trajectory(user_info.est_poses, "blue");              \
                                                                           \
                xlabel("X (m)");                                           \
                ylabel("Z (m)");                                           \
                zlabel("Y (m)");                                           \
                                                                           \
            }                                                              \
        }                                                                  \
    }                                                                      \
}

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


void mini_gt_reconstruction() {


	// Make Key
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
	//Make Mini-Cappella Graph


	// Generate GT path
	vector<Pose3> true_trajectory;
	Point3 initial_pos(0, 0, 2);

	Rot3 initial_rot = Rot3::Identity(); // along the +x axis
	Pose3 initial_pose(initial_rot, initial_pos);
	true_trajectory.push_back(initial_pose);

	double theta = 0.125; // rad
	// Rotation about the z-axis by some theta
	Matrix3 d_rot_mat;
	d_rot_mat << cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1;

	Pose3 d_pose(Rot3(d_rot_mat), Point3(1, 0, 0)); // change in pose is a rotation about z, and movement one unit along the local x-axis

	int N_poses = 25;
	for (int i = 0; i < N_poses; i++) {
		true_trajectory.push_back(d_pose * true_trajectory.back());
	}

	// Generate GT points that we would like to fuse to
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.0174533;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));

	vector<Pose3> gt_points;
	for (int i = 0; i < N_poses; i += 3) {
		gt_points.push_back(true_trajectory[i]);
	}



	// Generate drifted VIO poses
	vector<Pose3> vio_trajectory;
	Pose3 offset_vio(Rot3::Identity(), Point3(-0.5, 0.25, 0.2)); // Assume VIO starts at some initial offset, like is present in my U-rotated data
	vio_trajectory.push_back(offset_vio * initial_pose);

	// VIO Prior Noise Model

	//double os = 0.275;
	//double ps = 0.5; // Am I sure its orientation first, and not position? IT IS POSITION FIRST: SOURCE: https://github.com/haidai/gtsam/blob/master/examples/VisualISAMExample.cpp
	//noiseModel::Diagonal::shared_ptr a = noiseModel::Diagonal::Sigmas(Vector6(ps, ps, ps, os, os, os));

	// VIO noise model

	double vio_ori_stdev = 0.175; // rad->~10degrees
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));
	graph->addPrior<Pose3>(MK("x", 0), vio_trajectory[0], VIO_pose_noise_model);
	Values vals;
	vals.insert(MK("x", 0), vio_trajectory[0]);


	double theta_drift = 0.03;
	Matrix3 drift_rot_mat;
	drift_rot_mat << cos(theta_drift), -sin(theta_drift), 0,
		sin(theta_drift), cos(theta_drift), 0,
		0, 0, 1;
	Rot3 drift_rot(drift_rot_mat);

	Pose3 drift_vio(Rot3::Identity(), Point3(-0.1, 0.07, -0.07));

	for (int i = 1; i < N_poses; i++) {
		Pose3 vio_pose = (d_pose * drift_vio) * vio_trajectory.back();

		vals.insert(MK("x", i), vio_pose);
		// I think odometry should be a pose thats the PHYSICAL DIFFERENCE between two vio_poses!
		Pose3 odometry = vio_trajectory.back().between(vio_pose);
		graph->add(BetweenFactor<Pose3>(MK("x", i - 1), MK("x", i), odometry, VIO_pose_noise_model));


		vio_trajectory.push_back(vio_pose);
	}

	for (int i = 0; i < N_poses; i += 3) {
		graph->add(PriorFactor<Pose3>(MK("x", i), true_trajectory[i], GT_noise_model));
	}


	LevenbergMarquardtParams params;
	LevenbergMarquardtOptimizer optimizer(*graph, vals, params);

	// TODO: FIX this plotting code for the showing optimizer results, but I believe I've set up all priors / factors
	double last_error;
	do {
		last_error = optimizer.error();
		optimizer.iterate();

		vector<Pose3> est_trajectory;
		for (int i = 0; i < N_poses; i++) est_trajectory.push_back(optimizer.values().at<Pose3>(MK("x", i)));
		PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);

	} while (!checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol, last_error, optimizer.error()));

	show();

	cout << " Converged in " << optimizer.iterations() << " iterations, with " << optimizer.error() << " final error." << endl; // Currently doing 4 iterations



	PLOT(true_trajectory, gt_points, vio_trajectory, vio_trajectory);
	show();

	//GraphvizFormatting vizp;
	//vizp.plotFactorPoints = true;
	////vizp.mergeSimilarFactors = true;
	//vizp.binaryEdges = true;

	//graph->saveGraph("/home/admitriev/Research/gtsam_test/factor_graphs/factor_graph.dot", optimizer.values(), vizp);
	//graph->print();

	//show();
}

void mini_uwb_static_anchors() {


	// Make Key
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
	//Make Mini-Cappella Graph


	// Generate GT path
	vector<Pose3> true_trajectory;
	Point3 initial_pos(0, 0, 2);

	Rot3 initial_rot = Rot3::Identity(); // along the +x axis
	Pose3 initial_pose(initial_rot, initial_pos);
	true_trajectory.push_back(initial_pose);

	double theta = 0.125; // rad
	// Rotation about the z-axis by some theta
	Matrix3 d_rot_mat;
	d_rot_mat << cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1;

	Pose3 d_pose(Rot3(d_rot_mat), Point3(1, 0, 0)); // change in pose is a rotation about z, and movement one unit along the local x-axis

	int N_poses = 25;
	for (int i = 0; i < N_poses; i++) {
		true_trajectory.push_back(d_pose * true_trajectory.back());
	}

	
	// GT Noise model
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.0174533;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));

	// Discrete GT points
	vector<Pose3> gt_points;
	for (int i = 0; i < N_poses; i += 3) {
		gt_points.push_back(true_trajectory[i]);
	}

	// Generate drifted VIO poses
	vector<Pose3> vio_trajectory;
	Pose3 offset_vio(initial_rot, Point3(0,0,0)); // Assume VIO starts at some initial offset, like is present in my U-rotated data
	vio_trajectory.push_back(offset_vio * initial_pose);

	// VIO noise model
	//double vio_ori_stdev = 0.175; // rad->~10degrees
	//double vio_pos_stdev = 0.2;
	//double vio_ori_stdev = 0.075;
	//double vio_pos_stdev = 0.1;
	double vio_ori_stdev = 0.0075;
	double vio_pos_stdev = 0.01;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));


	// Seems the first VIO point gets moved very far off from where its supposed to be.
	// Adding a stronger prior on the first pose, does seem to keep our trajectory locked on to that point.
	// I think maybe this is just showing that one anchor isn't enough to properly constrain the trajectory

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


	// Generate UWB Anchor locations
	// Maybe just use one anchor for this example
	Point3 anchor(0, 0, 0);
	vals.insert(MK("a", 0), anchor);
	graph->addPrior<Point3>(MK("a", 0), anchor, noiseModel::Diagonal::Sigmas(Vector3(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev)));

	// UWB noise model
	double uwb_stdev = 0.1;
	// These actually both behave the same way
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev); // Apparently this is the correct noise model for a range
	//noiseModel::Diagonal::shared_ptr UWB_noise_model = noiseModel::Diagonal::Sigmas(Vector1(uwb_stdev));


	// Main loop !

	for (int i = 1; i < N_poses; i++) {

		vals.insert(MK("x", i), vio_trajectory[i]);
		Pose3 odometry = vio_trajectory.back().between(vio_trajectory[i]);
		graph->add(BetweenFactor<Pose3>(MK("x", i - 1), MK("x", i), odometry, VIO_pose_noise_model));

		if (i % 3 == 0) {
			// Add UWB ranging factor

			double true_distance = distance3(true_trajectory[i].translation(), anchor);
			graph->add(RangeFactor<Pose3, Point3, double>(MK("x", i), MK("a",0), true_distance, UWB_noise_model));

			//hold(on);
			//draw_vector(anchor, true_trajectory[i].translation(), "black");
		}
	}

	LevenbergMarquardtParams params;
	LevenbergMarquardtOptimizer optimizer(*graph, vals, params);

	double last_error;
	do {
		last_error = optimizer.error();
		optimizer.iterate();

		Values v = optimizer.values();

		vector<Pose3> est_trajectory;
		for (int i = 0; i < N_poses; i++) est_trajectory.push_back(v.at<Pose3>(MK("x", i)));
		PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);

		Marginals marg(*graph, v);

		cout << "a0 covariance: \n" << marg.marginalCovariance(MK("a", 0)) << endl;

	} while (!checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol, last_error, optimizer.error()));

	//show();

	cout << " Converged in " << optimizer.iterations() << " iterations, with " << optimizer.error() << " final error." << endl; // Currently doing 4 iterations


	vector<Pose3> est_trajectory;
	for (int i = 0; i < N_poses; i++) est_trajectory.push_back(optimizer.values().at<Pose3>(MK("x", i)));
	PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);
	show();

	//GraphvizFormatting vizp;
	//vizp.plotFactorPoints = true;
	////vizp.mergeSimilarFactors = true;
	//vizp.binaryEdges = true;

	//graph->saveGraph("/home/admitriev/Research/gtsam_test/factor_graphs/factor_graph.dot", optimizer.values(), vizp);
	//graph->print();

	//show();

	// Increasing anchors should bring us closer to GT
	// Increasing VIO noise , decreasing UWB noise should bring us closer to GT
}


int main(int argc, char* argv[]) {

	mini_uwb_static_anchors();

	return 0;
}
