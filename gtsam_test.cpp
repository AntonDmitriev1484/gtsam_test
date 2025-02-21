#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"


using namespace gtsam;
using namespace std;


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

	noiseModel::Diagonal::shared_ptr GT_noise_model_test = noiseModel::Diagonal::Sigmas(Vector3(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev));
	for (int i = 0; i < N_poses; i += 3) {
		// Cant use Points to initialize prior's on poses.
		graph->add(PriorFactor<Pose3>(MK("x", i), true_trajectory[i], GT_noise_model));
		//graph->add(PriorFactor<Point3>(MK("x", i), true_trajectory[i].translation(), GT_noise_model_test));
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
	//freopen("/dev/null", "w", stderr); // To mute Matplot++ errors in output stream

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


	// GT Noise model
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.0174533;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));

	// VIO noise model
	double vio_ori_stdev = 0.5; // 5.7deg
	double vio_pos_stdev = 0.5; // 5cm
	//double vio_ori_stdev = 0.1; // 5.7deg
	//double vio_pos_stdev = 0.05; // 5cm
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));


	// UWB noise model
	double uwb_stdev = 0.1;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	int N_poses = 25;
	vector<Pose3> true_trajectory, gt_points, vio_trajectory;
	gen_single_vio_trajectory_scenario(N_poses, true_trajectory, gt_points, vio_trajectory);


	graph->addPrior<Pose3>(MK("x", 0), vio_trajectory[0], GT_noise_model);
	Values vals;
	vals.insert(MK("x", 0), vio_trajectory[0]);

	// Generate UWB Anchor location(s)
	//vector<Point3> anchors = { Point3(-10, 0, 0), Point3(0,10,0), Point3(10,0,0), Point3(0, -10 , 20) };
	vector<Point3> anchors = { Point3(0,0,0) };
	for (int i = 0; i < anchors.size(); i++) {
		vals.insert(MK("a", i), anchors[i]);
		graph->add(NonlinearEquality<Point3>(MK("a", i), anchors[i]));
	}


	//ISAM2Params isam_params;
	//isam_params.factorization = ISAM2Params::QR;
	//isam_params.relinearizeSkip = 10;
	//ISAM2DoglegParams dogleg;
	//isam_params.optimizationParams = dogleg;
	//ISAM2* isam = new ISAM2(isam_params);


	// Main loop !
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

		//isam->update(*graph, vals);
		////graph->resize(0); // According to example
		////vals.clear(); // Still don't quite get why we need this vals.clear();


	}

	//LM_lambda_search(graph, vals, vio_trajectory, gt_points, true_trajectory);


	//LevenbergMarquardtParams lm_params;
	//lm_params.diagonalDamping = true;
	//lm_params.setlambdaInitial(1); // Poor initial estimates, require you to rely more on Gradient Descent to start lambda = 1,10
	//lm_params.lambdaFactor = 100000; 
	//// Note: you can also set verbosityLM: To monitor changes in lambda throughout the optimization process

	//// How much to change lambda by, based on an increase, or decrease in residual error.
	//// Increase error -> lambda* lambdafactor (rely on GD more)
	//// Decrease error -> lambda* lambdafactor (rely on GN more)
	//lm_params.linearSolverType = NonlinearOptimizerParams::LinearSolverType::MULTIFRONTAL_QR;
	//LevenbergMarquardtOptimizer lm_optimizer(*graph, vals, lm_params);

	////lm_optimizer.iterate();

	////GaussNewtonParams gn_params;
	////GaussNewtonOptimizer gn_optimizer(*graph, lm_optimizer.values(), gn_params);



	//int iter = 0;
	//double last_error;
	//do {
	//	last_error = lm_optimizer.error();
	//	lm_optimizer.iterate();

	//	Values v = lm_optimizer.values();

	//	vector<Pose3> est_trajectory;
	//	for (int i = 0; i < N_poses; i++) est_trajectory.push_back(v.at<Pose3>(MK("x", i)));
	//	PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);

	//	/*Marginals marg(*graph, v);

	//	cout << "------------" << endl;
	//	for (int i = 0; i < N_poses; i++) {
	//		if ( i == 10) cout << "x" << i << " \n" << marg.marginalCovariance(MK("x", i)) << ", ";
	//	}
	//	cout << endl;*/

	//	cout << "i: " << iter << " - error " << lm_optimizer.error() << endl;
	//	iter++;

	//} while (!checkConvergence(lm_params.relativeErrorTol, lm_params.absoluteErrorTol, lm_params.errorTol, last_error, lm_optimizer.error()));

	//cout << " Converged LM in " << lm_optimizer.iterations() << " iterations, with " << lm_optimizer.error() << " final error." << endl;


	//Values final_vals = lm_optimizer.values();
	//vector<Pose3> est_trajectory;
	//for (int i = 0; i < N_poses; i++) est_trajectory.push_back(final_vals.at<Pose3>(MK("x", i)));
	//PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);

	//show();

	////GraphvizFormatting vizp;
	////vizp.plotFactorPoints = true;
	//////vizp.mergeSimilarFactors = true;
	////vizp.binaryEdges = true;

	////graph->saveGraph("/home/admitriev/Research/gtsam_test/factor_graphs/factor_graph.dot", optimizer.values(), vizp);
	//graph->print();

	////show();
}

void mini_uwb_collaborative() {
	//freopen("/dev/null", "w", stderr); // To mute Matplot++ errors in output stream

	// Make Key
	const function<Key(int, int)> MK = [](int userid, int I) {
		Key k;
		char username;
		if (userid == -1) username = 'a';
		if (userid == 0)  username = 'x';
		if (userid == 1) username = 'y';
		k = symbol(username, I); // e.x. s11 if 'static11'

		return k;
	};

	NonlinearFactorGraph* graph = new NonlinearFactorGraph();


	// GT Noise model
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.0174533;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));

	// VIO noise model
	double vio_ori_stdev = 0.5; // 5.7deg
	double vio_pos_stdev = 0.5; // 5cm
	//double vio_ori_stdev = 0.1; // 5.7deg
	//double vio_pos_stdev = 0.05; // 5cm
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));


	// UWB noise model
	double uwb_stdev = 0.1;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	int N_poses = 25;
	vector<vector<Pose3>> true_trajectory, gt_points, vio_trajectory;
	gen_multi_user_vio_trajectory_scenario(N_poses, true_trajectory, gt_points, vio_trajectory);
	int N_users = true_trajectory.size();

	PLOT_MULTI_W_LM_PARAMS(N_users, true_trajectory, gt_points, vio_trajectory, vio_trajectory, 0, 0);

	Values vals;
	for (int usr = 0 ; usr < N_users; usr++) {
		graph->addPrior<Pose3>(MK(usr, 0), vio_trajectory[usr][0], GT_noise_model);
		vals.insert(MK(usr, 0), vio_trajectory[usr][0]);
	}


	// Generate UWB Anchor location(s)
	//vector<Point3> anchors = { Point3(-10, 0, 0), Point3(0,10,0), Point3(10,0,0), Point3(0, -10 , 20) };
	//vector<Point3> anchors = { Point3(0,0,0) };
	vector<Pose3> anchors = {};

	for (int i = 0; i < anchors.size(); i++) {
		vals.insert(MK(-1, i), anchors[i]);
		graph->add(NonlinearEquality<Pose3>(MK(-1, i), anchors[i]));
	}

	// Main loop !
	for (int i = 1; i < N_poses; i++) {

		// Add odometry factor to each user path
		for (int usr = 0; usr < N_users; usr++) {
			vals.insert(MK(usr, i), vio_trajectory[usr][i]);
			
			Pose3 odometry = vio_trajectory[usr].back().between(vio_trajectory[usr][i]);
			graph->add(BetweenFactor<Pose3>(MK(usr, i - 1), MK(usr, i), odometry, VIO_pose_noise_model));
		}
		

		if (i % 1 == 0) {
			// Add UWB ranging factor between each pose pair

			double true_distance = distance3(true_trajectory[0][i].translation(), true_trajectory[1][i].translation());
			graph->add(RangeFactor<Pose3, Pose3>(MK(0, i), MK(1, i), true_distance, UWB_noise_model));

			// And to whatever anchors exist

			for (int usr = 0; usr < N_users; usr++) {
				for (int j = 0; j < anchors.size(); j++) {
					double true_distance = distance3(true_trajectory[usr][i].translation(), anchors[j].translation());
					graph->add(RangeFactor<Pose3, Pose3>(MK(usr, i), MK(-1, j), true_distance, UWB_noise_model));
				}
			}

		}
 

	}

	graph->print();

	LM_lambda_search_multiuser_graph(graph, vals, vio_trajectory, gt_points, true_trajectory);
	//LM_lambda_search(graph, vals, vio_trajectory, gt_points, true_trajectory);


	//LevenbergMarquardtParams lm_params;
	//lm_params.diagonalDamping = true;
	//lm_params.setlambdaInitial(1); // Poor initial estimates, require you to rely more on Gradient Descent to start lambda = 1,10
	//lm_params.lambdaFactor = 100000; 
	//// Note: you can also set verbosityLM: To monitor changes in lambda throughout the optimization process

	//// How much to change lambda by, based on an increase, or decrease in residual error.
	//// Increase error -> lambda* lambdafactor (rely on GD more)
	//// Decrease error -> lambda* lambdafactor (rely on GN more)
	//lm_params.linearSolverType = NonlinearOptimizerParams::LinearSolverType::MULTIFRONTAL_QR;
	//LevenbergMarquardtOptimizer lm_optimizer(*graph, vals, lm_params);

	////lm_optimizer.iterate();

	////GaussNewtonParams gn_params;
	////GaussNewtonOptimizer gn_optimizer(*graph, lm_optimizer.values(), gn_params);



	//int iter = 0;
	//double last_error;
	//do {
	//	last_error = lm_optimizer.error();
	//	lm_optimizer.iterate();

	//	Values v = lm_optimizer.values();

	//	vector<Pose3> est_trajectory;
	//	for (int i = 0; i < N_poses; i++) est_trajectory.push_back(v.at<Pose3>(MK("x", i)));
	//	PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);

	//	/*Marginals marg(*graph, v);

	//	cout << "------------" << endl;
	//	for (int i = 0; i < N_poses; i++) {
	//		if ( i == 10) cout << "x" << i << " \n" << marg.marginalCovariance(MK("x", i)) << ", ";
	//	}
	//	cout << endl;*/

	//	cout << "i: " << iter << " - error " << lm_optimizer.error() << endl;
	//	iter++;

	//} while (!checkConvergence(lm_params.relativeErrorTol, lm_params.absoluteErrorTol, lm_params.errorTol, last_error, lm_optimizer.error()));

	//cout << " Converged LM in " << lm_optimizer.iterations() << " iterations, with " << lm_optimizer.error() << " final error." << endl;


	//Values final_vals = lm_optimizer.values();
	//vector<Pose3> est_trajectory;
	//for (int i = 0; i < N_poses; i++) est_trajectory.push_back(final_vals.at<Pose3>(MK("x", i)));
	//PLOT(true_trajectory, gt_points, est_trajectory, vio_trajectory);

	//show();

	////GraphvizFormatting vizp;
	////vizp.plotFactorPoints = true;
	//////vizp.mergeSimilarFactors = true;
	////vizp.binaryEdges = true;

	////graph->saveGraph("/home/admitriev/Research/gtsam_test/factor_graphs/factor_graph.dot", optimizer.values(), vizp);
	//graph->print();

	////show();
}

void iSAM_DL_run(
	NonlinearFactorGraph* graph, ISAM2* dl_isam, vector<vector<Pose3>>& est_trajectory,

	vector<vector<Pose3>> vio_trajectory, vector<vector<Pose3>> gt_points, vector<vector<Pose3>> true_trajectory,
	int poses_per_range, int N_poses, int N_users,
	noiseModel::Diagonal::shared_ptr GT_noise_model,
	noiseModel::Diagonal::shared_ptr VIO_noise_model,
	noiseModel::Diagonal::shared_ptr UWB_noise_model) {

	// Make Key
	const function<Key(int, int)> MK = [](int userid, int I) {
		Key k;
		char username;
		if (userid == -1) username = 'a';
		if (userid == 0)  username = 'x';
		if (userid == 1) username = 'y';
		k = symbol(username, I); // e.x. s11 if 'static11'

		return k;
	};

	Values initial_estimate;

	// Add priors
	for (int usr = 0; usr < N_users; usr++) {
		graph->addPrior<Pose3>(MK(usr, 0), vio_trajectory[usr][0], GT_noise_model);
		initial_estimate.insert(MK(usr, 0), vio_trajectory[usr][0]);
		vector<Pose3> et = { vio_trajectory[usr][0] };
		est_trajectory.push_back(et); // This may crash since the array hasnt been initd yet?
	}

	vector<Pose3> anchors = {};
	for (int i = 0; i < anchors.size(); i++) {
		initial_estimate.insert(MK(-1, i), anchors[i]);
		graph->add(NonlinearEquality<Pose3>(MK(-1, i), anchors[i]));
	}

	// Main loop !
	for (int i = 1; i < N_poses; i++) {

		// Add odometry factor to each user path
		for (int usr = 0; usr < N_users; usr++) {
			initial_estimate.insert(MK(usr, i), vio_trajectory[usr][i]);

			Pose3 odometry = vio_trajectory[usr].back().between(vio_trajectory[usr][i]);
			graph->add(BetweenFactor<Pose3>(MK(usr, i - 1), MK(usr, i), odometry, VIO_noise_model));
		}


		if (i % poses_per_range == 0) {
			// Add UWB ranging factor between each pose pair

			double true_distance = distance3(true_trajectory[0][i].translation(), true_trajectory[1][i].translation());
			graph->add(RangeFactor<Pose3, Pose3>(MK(0, i), MK(1, i), true_distance, UWB_noise_model));

			// And to whatever anchors exist

			for (int usr = 0; usr < N_users; usr++) {
				for (int j = 0; j < anchors.size(); j++) {
					double true_distance = distance3(true_trajectory[usr][i].translation(), anchors[j].translation());
					graph->add(RangeFactor<Pose3, Pose3>(MK(usr, i), MK(-1, j), true_distance, UWB_noise_model));
				}
			}

		}

		// https://github.com/haidai/gtsam/blob/master/examples/VisualISAM2Example.cpp
		// Update iSAM with the new factors
		dl_isam->update(*graph, initial_estimate);
		// Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
		// If accuracy is desired at the expense of time, update(*) can be called additional times
		// to perform multiple optimizer iterations every step.
		dl_isam->update();
		dl_isam->update();
		Values current_estimate = dl_isam->calculateEstimate();

		for (int usr = 0; usr < N_users; usr++) {
			est_trajectory[usr].push_back(current_estimate.at<Pose3>(MK(usr, i)));
		}

		// Clear the factor graph and values for the next iteration
		graph->resize(0);
		initial_estimate.clear();

	}
}


void iSAM_DL_hyperparameter_search() {
	// Make Key
	const function<Key(int, int)> MK = [](int userid, int I) {
		Key k;
		char username;
		if (userid == -1) username = 'a';
		if (userid == 0)  username = 'x';
		if (userid == 1) username = 'y';
		k = symbol(username, I); // e.x. s11 if 'static11'

		return k;
	};
	NonlinearFactorGraph* graph = new NonlinearFactorGraph();

	// Noise models
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.0174533;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	double vio_ori_stdev = 0.5; // 5.7deg
	double vio_pos_stdev = 0.5; // 5cm
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));
	double uwb_stdev = 0.1;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// Generate data
	int N_poses = 25;
	int N_users = 2;
	vector<vector<Pose3>> true_trajectory, gt_points, vio_trajectory;
	gen_multi_user_vio_trajectory_scenario(N_poses, true_trajectory, gt_points, vio_trajectory);

	//// Run 1
	//vector<double> attempt_lambdaInitial = { 10, 1, 0.1, 0.001, 0.0001, 0.00001 };
	//vector<double> attempt_lambdaFactor = { 100000, 10000, 1000, 100, 10, 7, 5, 3 }; // Won't run with 1

	// Set up D-iSam instance
	ISAM2Params isam_params;
	isam_params.factorization = ISAM2Params::QR;
	isam_params.relinearizeThreshold = 0.1;
	isam_params.relinearizeSkip = 10;
	ISAM2DoglegParams dogleg;
	isam_params.optimizationParams = dogleg;
	ISAM2* isam = new ISAM2(isam_params);

	// Call function to run online optimization

	vector<vector<Pose3>> est_trajectory; // Will be filled as data is replayed
	iSAM_DL_run(graph, isam, est_trajectory,
		vio_trajectory, gt_points, true_trajectory,
		1, N_poses, N_users,
		GT_noise_model, VIO_pose_noise_model, UWB_noise_model);


	// Plot results
	PLOT_MULTI(N_users, true_trajectory, gt_points, est_trajectory, vio_trajectory)
	//PLOT_MULTI_W_LM_PARAMS(N_users, true_trajectory, gt_points, est_trajectory, vio_trajectory, lambdaInitial, lambdaFactor);

	show();
}





int main(int argc, char* argv[]) {

	//mini_uwb_static_anchors();
	//mini_gt_reconstruction();
	//mini_uwb_collaborative();

	iSAM_DL_hyperparameter_search();

	return 0;
}
